"""SSH deployment transport. Credentials come from the environment or a prompt."""
import argparse
import getpass
import os
import posixpath
import select
from pathlib import Path

import paramiko


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('action', choices=['run', 'python', 'put', 'get', 'sync'])
    parser.add_argument('source')
    parser.add_argument('destination', nargs='?')
    parser.add_argument('--timeout', type=int, default=60)
    args = parser.parse_args()
    if args.action in ('put', 'get', 'sync') and not args.destination:
        parser.error('destination is required for put/get/sync')
    client = paramiko.SSHClient()
    client.load_system_host_keys()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    client.connect(os.environ.get('CAR_SSH_HOST', '10.206.170.35'),
                   username=os.environ.get('CAR_SSH_USER', 'jetson'),
                   password=os.environ.get('CAR_SSH_PASSWORD') or getpass.getpass('Car SSH password: '),
                   timeout=20, banner_timeout=20, auth_timeout=20,
                   look_for_keys=False, allow_agent=False)
    client.get_transport().set_keepalive(10)
    try:
        if args.action in ('run', 'python'):
            command = 'bash -s' if args.action == 'run' else 'python3 -'
            stdin, stdout, stderr = client.exec_command(command, timeout=args.timeout)
            stdin.write(Path(args.source).read_text(encoding='utf-8'))
            stdin.channel.shutdown_write()
            channel = stdout.channel
            while not channel.exit_status_ready() or channel.recv_ready() or channel.recv_stderr_ready():
                select.select([channel], [], [], 1.0)
                if channel.recv_ready():
                    print(channel.recv(65536).decode('utf-8', 'replace'), end='', flush=True)
                if channel.recv_stderr_ready():
                    print(channel.recv_stderr(65536).decode('utf-8', 'replace'), end='', flush=True)
            raise SystemExit(channel.recv_exit_status())
        with client.open_sftp() as sftp:
            if args.action == 'get':
                output = Path(args.destination)
                output.parent.mkdir(parents=True, exist_ok=True)
                sftp.get(args.source, str(output))
                print(str(output), output.stat().st_size)
            elif args.action == 'put':
                sftp.put(args.source, args.destination)
                print(args.destination)
            else:
                root = Path(args.source).resolve()
                count = 0
                for path in sorted(root.rglob('*')):
                    if '__pycache__' in path.parts or path.suffix == '.pyc':
                        continue
                    remote = posixpath.join(args.destination, path.relative_to(root).as_posix())
                    if path.is_dir():
                        try:
                            sftp.mkdir(remote)
                        except IOError:
                            sftp.stat(remote)
                    elif path.is_file():
                        data = path.read_bytes()
                        if path.suffix in ('.py', '.sh'):
                            data = data.replace(b'\r\n', b'\n')
                        try:
                            with sftp.file(remote, 'rb') as existing:
                                if existing.read() == data:
                                    continue
                        except IOError:
                            pass
                        with sftp.file(remote, 'wb') as handle:
                            handle.write(data)
                        if path.suffix in ('.py', '.sh'):
                            sftp.chmod(remote, 0o755)
                        count += 1
                print('Synced %d files to %s' % (count, args.destination))
    finally:
        client.close()


if __name__ == '__main__':
    main()
