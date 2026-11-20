# Forklift pallet engagement

Run three scripted checks in MuJoCo with a rear-steered forklift and a stationary rackable pallet. The probe checks steering direction, a centered insertion to 600 mm, and contact after a 40 mm lateral offset.

## Run the checks

From the repository root on Linux or macOS, use Python 3.12 with virtual environment support and pip.

```sh
python3 -m venv .venv && .venv/bin/python -m pip install -r requirements.txt && .venv/bin/python probe.py
```

The probe prints the result of each check and exits with status zero when all three pass. The offset check passes when the fork hits the pallet.

## View the scene

On a machine with a graphical display, open the bundled model in the MuJoCo desktop viewer.

```sh
.venv/bin/python -m mujoco.viewer --mjcf=forklift_c.xml
```

The viewer opens the scene for interactive inspection. The scripted checks run through `probe.py`.

## Asset credits

NVIDIA's Isaac Sim 6.2 ForkliftC supplies the forklift geometry. The MuJoCo adaptation uses simplified materials, estimated inertias, and rear Ackermann steering commands. [ForkliftC source description](https://docs.isaacsim.omniverse.nvidia.com/6.2.0/controllers/tutorial_ackermann.html) · [NVIDIA asset terms](https://docs.isaacsim.omniverse.nvidia.com/6.2.0/common/license-isaac-sim-additional.html)

NVIDIA's SimReady Warehouse 01 supplies the black rackable pallet. The MuJoCo adaptation uses a flat material and convex collision pieces. [Original pallet](https://huggingface.co/datasets/nvidia/PhysicalAI-SimReady-Warehouse-01/tree/c7fe115cb79c7ddbd0532630d7768b5736b0ecc4/Props/general/SM_RackablePallet_A01_BlackA_01) · [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/)
