import argparse
import sys

from src.app import App, DevelopmentApp


def parse_args(argv):
  parser = argparse.ArgumentParser()
  parser.add_argument(
    "--development",
    action="store_true",
    help="run in interactive mode (no GPIO/SMBus required)"
  )
  return parser.parse_args(argv)


if __name__ == "__main__":
  args = parse_args(sys.argv[1:])
  if args.development:
    DevelopmentApp().run()
  else:
    App().run()
