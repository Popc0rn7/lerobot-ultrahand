from lerobot.teleoperators.ultrahand.simulate import UltrahandSimulator

import argparse


def main():
    parser = argparse.ArgumentParser(description="manual to this script")
    parser.add_argument(
        "--port", type=str, default="/dev/ttyUSB0"
    )  # 设置默认值为字符 0，不设置默认值则为 None
    parser.add_argument(
        "--model_path", type=str, default="lerobot/model/uh-right/uh-right.xml"
    )
    args = parser.parse_args()

    simulator = UltrahandSimulator(port=args.port, model_path=args.model_path)
    simulator.run_viewer()


if __name__ == "__main__":
    main()
