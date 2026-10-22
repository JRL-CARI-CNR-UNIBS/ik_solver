#!/usr/bin/env python3
"""Timing benchmark of the services of an ik_solver node.

Measures the wall-clock time of each service call, seen from the client (request
serialization + transport + computation + response), and prints mean, std, min,
median, p95 and max for each test.

- get_fk, get_ik: N single calls (default 1000) on random configurations within bounds.
- get_fk_array, get_ik_array, get_task_redundant_ik_array: R calls (default 10) for each
  payload size (default 10, 100, 1000, 10000 configurations/targets).

IK targets are the FK poses of random configurations, so they are always reachable.
By default no seeds are sent (the true configuration as a seed would make IK trivial);
use --with-seeds to send it anyway.

Usage:
  ros2 run ik_solver_test test_service_timing.py <namespace> [options]

Examples:
  ros2 run ik_solver_test test_service_timing.py solver1
  ros2 run ik_solver_test test_service_timing.py solver1 --services get_fk_array get_ik_array
  ros2 run ik_solver_test test_service_timing.py solver1 --payloads 10 100 --csv /tmp/timing.csv
"""

import argparse
import csv
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node

from ik_solver_msgs.msg import Configuration, IkTarget
from ik_solver_msgs.srv import GetBound, GetFk, GetFkArray, GetFrames, GetIk, GetIkArray

SINGLE_SERVICES = ['get_fk', 'get_ik']
ARRAY_SERVICES = ['get_fk_array', 'get_ik_array', 'get_task_redundant_ik_array']


def compute_stats(times_s):
    """Return a dict of statistics (in milliseconds) of a list of durations in seconds."""
    t = np.asarray(times_s) * 1e3
    if t.size == 0:
        return None
    return {
        'n': int(t.size),
        'mean': float(np.mean(t)),
        'std': float(np.std(t, ddof=1)) if t.size > 1 else 0.0,
        'min': float(np.min(t)),
        'median': float(np.median(t)),
        'p95': float(np.percentile(t, 95)),
        'max': float(np.max(t)),
        'total': float(np.sum(t)),
    }


class ServiceTimingNode(Node):
    def __init__(self, ns, timeout):
        super().__init__('ik_solver_service_timing')
        self.ns = ns
        self.timeout = timeout

        res = self.call(self.client(GetBound, 'get_bounds'), GetBound.Request())
        self.joint_names = [b.joint_name for b in res.boundaries]
        self.lb = np.array([b.lower_bound for b in res.boundaries])
        self.ub = np.array([b.upper_bound for b in res.boundaries])
        self.get_logger().info(f'Joints: {self.joint_names}')

        res = self.call(self.client(GetFrames, 'get_frames'), GetFrames.Request())
        self.base_frame = res.base_frame
        self.tool_frame = res.tool_frame
        self.get_logger().info(f'Frames: base={self.base_frame}, tool={self.tool_frame}')

    # ---------------------------------------------------------------- helpers
    def client(self, srv_type, name):
        cli = self.create_client(srv_type, f'/{self.ns}/{name}')
        while not cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'Service /{self.ns}/{name} not available, waiting again...')
        return cli

    def call(self, cli, req):
        future = cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=self.timeout)
        if not future.done():
            cli.remove_pending_request(future)
            raise TimeoutError(f'{cli.srv_name} did not answer within {self.timeout} s')
        return future.result()

    def timed_call(self, cli, req):
        t0 = time.perf_counter()
        res = self.call(cli, req)
        return time.perf_counter() - t0, res

    def random_configurations(self, n):
        return self.lb + np.random.rand(n, len(self.joint_names)) * (self.ub - self.lb)

    def fk_array_request(self, q):
        req = GetFkArray.Request()
        req.joint_names = self.joint_names
        req.reference_frame = self.base_frame
        req.tip_frame = self.tool_frame
        req.configurations = [Configuration(configuration=row.tolist()) for row in q]
        return req

    def ik_array_request(self, q, poses, with_seeds):
        req = GetIkArray.Request()
        req.seed_joint_names = self.joint_names
        for row, pose in zip(q, poses):
            target = IkTarget()
            target.pose.header.frame_id = self.base_frame
            target.pose.pose = pose
            if with_seeds:
                target.seeds = [Configuration(configuration=row.tolist())]
            req.targets.append(target)
        return req

    # ------------------------------------------------------------------ tests
    def bench_fk(self, n_trials):
        cli = self.client(GetFk, 'get_fk')
        times, poses, qs = [], [], self.random_configurations(n_trials)
        for i, q in enumerate(qs):
            req = GetFk.Request()
            req.joint_names = self.joint_names
            req.reference_frame = self.base_frame
            req.tip_frame = self.tool_frame
            req.configuration.configuration = q.tolist()
            dt, res = self.timed_call(cli, req)
            times.append(dt)
            poses.append(res.pose)
            if (i + 1) % 100 == 0:
                self.get_logger().info(f'get_fk: {i + 1}/{n_trials}')
        return times, qs, poses

    def bench_ik(self, qs, poses, with_seeds):
        cli = self.client(GetIk, 'get_ik')
        times, n_fail = [], 0
        for i, (q, pose) in enumerate(zip(qs, poses)):
            req = GetIk.Request()
            req.target.pose = pose
            req.seed_joint_names = self.joint_names
            if with_seeds:
                req.target.seeds = [Configuration(configuration=q.tolist())]
            dt, res = self.timed_call(cli, req)
            times.append(dt)
            if len(res.solution.configurations) == 0:
                n_fail += 1
            if (i + 1) % 100 == 0:
                self.get_logger().info(f'get_ik: {i + 1}/{len(qs)}')
        return times, n_fail

    def bench_array(self, service, payload, repetitions, with_seeds):
        if service == 'get_fk_array':
            cli = self.client(GetFkArray, service)
            fk_cli = None
        else:
            cli = self.client(GetIkArray, service)
            fk_cli = self.client(GetFkArray, 'get_fk_array')

        times, n_fail = [], 0
        for r in range(repetitions):
            q = self.random_configurations(payload)
            fk_req = self.fk_array_request(q)
            if fk_cli is None:
                req = fk_req
            else:
                # target poses are generated with an (untimed) get_fk_array call
                poses = self.call(fk_cli, fk_req).poses.poses
                req = self.ik_array_request(q, poses, with_seeds)

            dt, res = self.timed_call(cli, req)
            times.append(dt)
            if fk_cli is not None:
                n_fail += sum(1 for s in res.solutions if len(s.configurations) == 0)
            self.get_logger().info(
                f'{service} payload={payload}: rep {r + 1}/{repetitions} -> {dt * 1e3:.2f} ms')
        return times, n_fail


def print_table(rows):
    header = (f'{"service":<28} {"payload":>8} {"n":>5} {"mean":>11} {"std":>11} {"min":>11} '
              f'{"median":>11} {"p95":>11} {"max":>11} {"per item":>11} {"fail":>7}')
    print()
    print('Service call times [ms] (per item = mean / payload)')
    print(header)
    print('-' * len(header))
    for r in rows:
        print(f'{r["service"]:<28} {r["payload"]:>8} {r["n"]:>5} {r["mean"]:>11.3f} {r["std"]:>11.3f} '
              f'{r["min"]:>11.3f} {r["median"]:>11.3f} {r["p95"]:>11.3f} {r["max"]:>11.3f} '
              f'{r["per_item"]:>11.4f} {r["failures"]:>7}')
    print()


def main():
    rclpy.init(args=sys.argv)
    argv = rclpy.utilities.remove_ros_args(sys.argv)[1:]

    parser = argparse.ArgumentParser(description='Timing benchmark of the ik_solver services.')
    parser.add_argument('namespace', help='namespace of the ik_solver node (e.g. solver1)')
    parser.add_argument('--services', nargs='+', choices=SINGLE_SERVICES + ARRAY_SERVICES,
                        default=SINGLE_SERVICES + ARRAY_SERVICES, help='services to test (default: all)')
    parser.add_argument('--trials', type=int, default=1000, help='calls for get_fk and get_ik (default: 1000)')
    parser.add_argument('--payloads', type=int, nargs='+', default=[10, 100, 1000, 10000],
                        help='payload sizes for the *_array services (default: 10 100 1000 10000)')
    parser.add_argument('--repetitions', type=int, default=10,
                        help='calls per payload for the *_array services (default: 10)')
    parser.add_argument('--with-seeds', action='store_true',
                        help='send the sampled configuration as IK seed')
    parser.add_argument('--timeout', type=float, default=600.0, help='timeout of each call in s (default: 600)')
    parser.add_argument('--csv', help='optional path of a CSV file where the statistics are saved')
    parser.add_argument('--seed', type=int, help='seed of the random configuration generator')
    args = parser.parse_args(argv)

    if args.seed is not None:
        np.random.seed(args.seed)

    node = ServiceTimingNode(args.namespace, args.timeout)
    rows = []

    def add_row(service, payload, times, failures):
        stats = compute_stats(times)
        stats.update(service=service, payload=payload, failures=failures,
                     per_item=stats['mean'] / payload)
        rows.append(stats)

    try:
        if 'get_fk' in args.services or 'get_ik' in args.services:
            fk_times, qs, poses = node.bench_fk(args.trials)
            if 'get_fk' in args.services:
                add_row('get_fk', 1, fk_times, 0)
            if 'get_ik' in args.services:
                ik_times, ik_fail = node.bench_ik(qs, poses, args.with_seeds)
                add_row('get_ik', 1, ik_times, ik_fail)

        for service in ARRAY_SERVICES:
            if service not in args.services:
                continue
            for payload in args.payloads:
                times, failures = node.bench_array(service, payload, args.repetitions, args.with_seeds)
                add_row(service, payload, times, failures)
    except (TimeoutError, KeyboardInterrupt) as e:
        node.get_logger().error(f'Benchmark interrupted: {e!r}. Printing partial results.')

    print_table(rows)

    if args.csv and rows:
        keys = ['service', 'payload', 'n', 'mean', 'std', 'min', 'median', 'p95', 'max',
                'total', 'per_item', 'failures']
        with open(args.csv, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=keys)
            writer.writeheader()
            writer.writerows([{k: r[k] for k in keys} for r in rows])
        node.get_logger().info(f'Statistics saved in {args.csv}')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
