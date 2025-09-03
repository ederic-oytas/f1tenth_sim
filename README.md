
# f1tenth_sim - VS Code Integration Using Dev Containers

This repository provides a dev container workspace for the [UNLV F1 course](https://github.com/unlv-f1) course.

This README is a work in progress.

## Installation

First, install each of the following:

- [VS Code](https://code.visualstudio.com/)
- [Docker](https://www.docker.com/)
- [VS Code Dev Containers](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers)

> **Tip**: Consider configuring Docker to have it start on system startup, so you don't need to manually start the Docker daemon.

Next, clone this repository onto your machine:

```
git clone --recurse-submodules https://github.com/ederic-oytas/f1tenth_sim.git
```

Afterward, open the folder in VS Code. If you added VS Code to the command line, you can use:

```
code f1tenth_sim
```

You should see VS Code open the folder **locally** on your computer, as shown below:

[IMAGE]

Next, we will enter the container. Use the Command Palette (`Ctrl+Shift+P` or View > Command Palette) and click "Dev Containers: Reopen in Container", as shown below:

[IMAGE]

VS Code should build and open the container. Once it's finished building, VS Code will open the container as shown below:

[IMAGE]
