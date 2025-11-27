import sys


def greet(name: str, project: str) -> None:
    if "detection" in project.lower() or "object" in project.lower():
        print(f"\nHello {name}, I'm Ben and I'll also be working on object detection!!")
    else:
        print(
            f"\nHello {name}, it's nice to meet you. My name is Ben and I'll be working on object detection"
        )

    with open(__file__, "a") as f:
        _ = f.write(f"# {name} : {project}\n")


if __name__ == "__main__":
    if len(sys.argv) < 3:
        name = None
        project = None
        while not name or not project:
            userdata = input("Enter a name and a project name: ")
            if not userdata or len(userdata.split()) < 2:
                print("You need to enter a name and a project name \n")
                continue
            name = userdata.split()[0]
            project = " ".join(userdata.split()[1:])

    else:
        name = sys.argv[1]
        project = " ".join(sys.argv[2:])

    greet(name, project)


# Log of people and their projects:
# Benjamin : Object Detection
