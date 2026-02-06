# 🖼️ ROS 2 Image Publisher & Subscriber

This project demonstrates a simple image publisher and subscriber using **ROS 2** and **Python**.

The repository is **portable** and can be cloned into **any folder**.  
All project paths are **relative**, except for the ROS 2 installation path.

---

## 📥 Clone the Repository

Clone the repository using Git:

```bash
git clone <repository-url>
```

Navigate into the cloned folder:

```bash
cd <path-to-cloned-repo>
```

You can clone this repository **anywhere** on your system.

---

## ⚙️ Dependencies / Prerequisites

### Common (All Operating Systems)
- Python 3.8 or newer
- ROS 2 (Jazzy or compatible)
- Git

### Windows
- Windows 10 or newer
- Pixi
- ROS 2 Windows binary installation

### Linux
- Ubuntu 22.04 or newer
- ROS 2 installed via apt

### macOS
- macOS 12 or newer
- ROS 2 installed via Homebrew or source build

---

## 🛠️ Installation

### 🪟 Windows (Pixi-based ROS 2)

Two paths are important:

- `<ros-workspace>`  
  ROS 2 Pixi workspace  
  Example: `C:\pixi_ws`

- `<project-path>`  
  Path where this repository is cloned  
  (can be any folder)

---

### 🐧 Linux

Install ROS 2:

```bash
sudo apt install ros-<ros-distro>-desktop
```

Source ROS 2:

```bash
source /opt/ros/<ros-distro>/setup.bash
```

---

### 🍎 macOS

Install ROS 2 using Homebrew or from source.

Source ROS 2:

```bash
source <ros-install-path>/setup.bash
```

---

## ▶️ How to Run the Project

You need **two terminals**:
- One for the publisher
- One for the subscriber

---

### 🪟 Windows

#### Terminal 1 — Publisher

```bat
cd <ros-workspace>
pixi shell
call <ros-workspace>\ros2-windows\local_setup.bat

cd /d <project-path>
set ROS_DOMAIN_ID=0
python publisher\image_publisher.py
```

#### Terminal 2 — Subscriber

```bat
cd <ros-workspace>
pixi shell
call <ros-workspace>\ros2-windows\local_setup.bat

cd /d <project-path>
set ROS_DOMAIN_ID=0
python subscriber\image_subscriber.py
```

---

### 🐧 Linux / 🍎 macOS

In **both terminals**:

```bash
source /opt/ros/<ros-distro>/setup.bash
cd <project-path>
export ROS_DOMAIN_ID=0
```

Terminal 1:

```bash
python publisher/image_publisher.py
```

Terminal 2:

```bash
python subscriber/image_subscriber.py
```

---

## 📂 Output

All output is generated relative to the project directory.

- Images are saved in:
  ```
  output/images/
  ```

- Metadata is saved in:
  ```
  output/metadata.json
  ```

---

## 📝 Notes

- The repository can be cloned to **any folder**
- No project-specific absolute paths are required
- Only the ROS 2 installation path depends on the system
