# Image Publisher & Subscriber (ROS 2 Jazzy -- Windows)

This project demonstrates a simple **image publisher and subscriber**
using **ROS 2 Jazzy** on Windows.

The repository can be cloned **to any folder** on your system.\
All project paths are **relative**, so no fixed directory is required.

------------------------------------------------------------------------

## Prerequisites

-   Windows OS\
-   ROS 2 Jazzy installed using Pixi\
-   Python installed and available in PATH

Follow the setup instructions in:
installation/ROS2_Jazzy_Windows_Setup_Guide.pdf

------------------------------------------------------------------------

## Project Structure

project-root/ ├── publisher/ │ └── image_publisher.py ├── subscriber/ │
└── image_subscriber.py ├── output/ │ ├── images/ │ └── metadata.json
└── installation/ └── ROS2_Jazzy_Windows_Setup_Guide.pdf

------------------------------------------------------------------------

## How to Run the Project

You need **two terminals**.

### Step 1: Set up ROS 2 environment (both terminals)

cd C:`\pixi`{=tex}\_ws\
pixi shell\
call C:`\pixi`{=tex}\_ws`\ros2`{=tex}-windows`\local`{=tex}\_setup.bat

------------------------------------------------------------------------

### Step 2: Navigate to the project folder

cd `<path-to-cloned-repository>`{=html}

Example: cd D:`\projects`{=tex}`\image`{=tex}-pub-sub

------------------------------------------------------------------------

### Step 3: Set ROS Domain ID (both terminals)

set ROS_DOMAIN_ID=0

------------------------------------------------------------------------

### Step 4: Run Publisher (Terminal 1)

python publisher`\image`{=tex}\_publisher.py

------------------------------------------------------------------------

### Step 5: Run Subscriber (Terminal 2)

python subscriber`\image`{=tex}\_subscriber.py

------------------------------------------------------------------------

## Output

-   Images are saved in: output/images/

-   Metadata is saved in: output/metadata.json

All output paths are **relative to the project folder**.

------------------------------------------------------------------------

## Notes

-   Publisher and Subscriber must run in **separate terminals**
-   The repository can be cloned to **any location**
-   Only the ROS 2 installation path is fixed
