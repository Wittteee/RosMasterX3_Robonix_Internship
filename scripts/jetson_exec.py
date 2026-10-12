"""Run a reviewed shell script over SSH; credentials stay in the environment."""
from __future__ import annotations

import argparse
import getpass
import os
from pathlib import Path
import sys

import paramiko


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("script", type=Path)
    parser.add_argument("--host", default="10.206.9.102")
    parser.add_argument("--user", default="jetson")
    parser.add_argument("--upload", nargs=2, action="append", default=[], metavar=("LOCAL", "REMOTE"))
    parser.add_argument("--download", nargs=2, action="append", default=[], metavar=("REMOTE", "LOCAL"))
    args = parser.parse_args()
    sys.stdout.reconfigure(encoding="utf-8")
    client = paramiko.SSHClient()
    client.load_system_host_keys()
    try:
        client.connect(args.host, username=args.user,
                       password=os.environ.get("X3_SSH_PASSWORD") or getpass.getpass("SSH password: "),
                       timeout=10, auth_timeout=10, look_for_keys=False, allow_agent=False)
        if args.upload:
            with client.open_sftp() as sftp:
                for local_path, remote_path in args.upload:
                    if not remote_path.startswith("/home/" + args.user + "/") or "/../" in remote_path:
                        raise ValueError("Upload destination must stay inside the SSH user's home")
                    sftp.put(local_path, remote_path)
        channel = client.get_transport().open_session(timeout=10)
        channel.set_combine_stderr(True)
        channel.exec_command("bash -s")
        channel.sendall(args.script.read_bytes())
        channel.shutdown_write()
        while True:
            chunk = channel.recv(65536)
            if not chunk:
                break
            print(chunk.decode("utf-8", errors="replace"), end="", flush=True)
        result = channel.recv_exit_status()
        if args.download:
            with client.open_sftp() as sftp:
                for remote_path, local_path in args.download:
                    destination = Path(local_path)
                    destination.parent.mkdir(parents=True, exist_ok=True)
                    sftp.get(remote_path, str(destination))
        return result
    finally:
        client.close()


if __name__ == "__main__":
    raise SystemExit(main())
