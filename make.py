import os
import argparse

class main():
    def __init__(self) -> None:
        self.ACTIONS = {
            "build": self._build,
            "start": self._start,
            "stop": self._stop
        }

        self.parser = argparse.ArgumentParser(
            prog = "make",
            description = "",
            epilog = ""
        )
        
        self.parser.add_argument("action")

        self._parse_arguments()

    def _parse_arguments(self):
        arguemnts = self.parser.parse_args()

        try:
            self.ACTIONS.get(arguemnts.action)()
        except:
            print("Invalid action argument")

    def _build(self):
        # Pull and update git submodules to the latest version
        os.system("git submodule init")
        os.system("git submodule update")

        # Build images and create the environments
        os.system("docker compose build")
        os.system("docker compose create")

    def _start(self):
        os.system("docker compose start")

    def _stop(self):
        os.system("docker compose stop")

if __name__ == "__main__":
    main()
