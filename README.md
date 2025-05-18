# HobbyBot - 6-Axis Robotic Arm Project

## 🚀 Project Overview

HobbyBot is an open-source, cost-effective, and fully functional 6-axis robotic arm designed for hobbyists and makers. The project combines advanced hardware and software components, making it a versatile platform for experimentation, learning, and prototyping.

The robotic arm is powered by an STM32WB55RG microcontroller and features precise servo control using the PCA9685. It also incorporates power management via TPS2121 and is designed for easy customization and future improvements. The project is modular, allowing for the integration of additional features like Bluetooth control.

---

## 🌟 Features

* **6-Axis Robotic Arm**: High-precision control and smooth movement.
* **Modular and Flexible Design**: Easily upgrade or modify components.
* **Bluetooth Connectivity (Planned)**: Future support for wireless control via an app.
* **Efficient Power Management**: USB-C Power Delivery and automatic source switching.
* **Custom Servo Driver Library**: Developed specifically for the STM32WB55RG.
* **Open-Source and Community-Friendly**: Contributions and modifications are encouraged.

---

## 🛠️ Installation

1. **Clone the Repository:**

   ```bash
   git clone https://github.com/Leonhard17/HobbyBot.git
   ```
2. **Navigate to the Project Directory:**

   ```bash
   cd HobbyBot
   ```
3. **Install Dependencies (if applicable):**

   ```bash
   pip install -r requirements.txt
   ```
4. **Setup Git LFS:**

   ```bash
   git lfs install
   ```

---

## 🚦 Usage

* Connect the robotic arm to your PC via USB-C.
* Launch the control application (Windows executable):

  ```bash
  ./DesktopApp/RoboterArm.exe
  ```
* Follow on-screen instructions for basic movement and control.

### Bluetooth Mode (Planned)

* Enable Bluetooth on your device.
* Open the control app and select Bluetooth connection mode.

---

## 💻 Technologies Used

* **STM32WB55RG Microcontroller**: Embedded control system.
* **PCA9685 PWM Driver**: Precise servo control.
* **TPS2121 Power Path Controller**: Automatic power source management.
* **Python (DesktopApp)**: Compiled into a Windows executable for control.
* **STM32CubeIDE & CubeMX**: Firmware development environment.
* **Autodesk Inventor**: Mechanical design and simulation.

---

## 📂 Project Structure

```
HobbyBot/
├── DesktopApp/             # Windows executable control application
├── Mechanical_CAD_Files/   # Autodesk Inventor .ipt and .iam files
├── PCB_CAD_Files/          # PCB schematics and layouts
├── STM32_DemoCode/         # Embedded firmware for STM32WB55RG
├── LICENSE                 # MIT License
└── README.md               # Project overview and instructions
```

---

## 📜 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

---

## 📧 Contact

Created by **Leonhard Waibl** - Feel free to reach out via GitHub issues or discussions for questions, contributions, or collaboration opportunities.

📧 Email: [leonhardwaibl@gmail.com](mailto:leonhardwaibl@gmail.com)
