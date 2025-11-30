# main.py

from navigation_system import NavigationSystem
import json

def main():
    # Load config
    with open("Data/configs/config.json") as f:
        cfg = json.load(f)

    print(cfg)

    # Create system orchestrator
    system = NavigationSystem(cfg)

    # Run entire pipeline
    system.run()

if __name__ == "__main__":
    main()