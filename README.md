# Passivity-based-Whole-Body-Control-control
EiR Lanari G1

---

Passivity-based Whole-Body Control control. 

Simulate a humanoid controlled via Model Predictive control. 
The project should modify the he provided humanoid whole body controller with the passivity-based one.

- Englsberger et al., 2020, "MPTC – Modular Passive Tracking Controller for
stack of tasks based control frameworks”.

Contact Dr. Nicola Scianca for details
- paper(s) in attachment

- deadline: July 2025

Update the summarized methods and the actual implementation
Notion links:
 MPTC:
 https://marred-pizza-2fb.notion.site/MPTC-Modular-Passive-Tracking-Controller-for-stack-of-tasks-based-control-frameworks-19fbdb3887ff80c49adaff3c1f9e6ba4?pvs=4



## 🔧 Setup & Requirements

You need Python (preferably 3.11 or 3.12) and the following dependencies:

Many packages are shared with this repo:  
🔗 [https://github.com/DIAG-Robotics-Lab/ismpc](https://github.com/DIAG-Robotics-Lab/ismpc/blob/main/README.md)

**Important version constraints:**
- `numpy==1.26.0`
- `pin==2.7.0` (higher versions may be incompatible with `dartpy` / `cmeel-boost`/'numpy)
- `dartpy==0.2.0` (you may need Python 3.12 to install it correctly)

### Install using pip

```bash
pip install numpy==1.26.0
pip install dartpy casadi scipy matplotlib osqp
pip install pin==2.7.0
```
In additional (not mandatory) if want to use meshcat viosualization also run 
```
pip install meshcat
```

For more detail see the requirement.tx file or if you are using a virtual enveiroment you could just run 

```
pip install -r requirements.txt
```

## Run the simulation
```
python3 project/simulation.py
```
then press spacebar to start it