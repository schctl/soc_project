# Driver Drowsiness Detector IP

Vibecoded Driver Drowsiness Detection System. Implementation based on [this](https://ieeexplore.ieee.org/document/1563673/) paper titled _"A FPGA based driver drowsiness detecting system"_, by Fei Wang and Huabiao Qin.

## Setup

Install [verilator](https://www.veripool.org/verilator/) to run simulations. You'll also need `build-essential` to run `make`.

```
# apt install build-essential verilator
```

Install `stb_image`.

```sh
$ bash scripts/get_stb.sh
```

Download any drowsy driver dataset (for example, [this one](https://www.kaggle.com/datasets/ismailnasri20/driver-drowsiness-dataset-ddd)), and place it into `dataset/DDD/drowsy` and `dataset/DDD/alert`.

## Usage

Call `make` with the `run` target and an input image specified.

```sh
$ make run IMG=dataset/DDD/drowsy/A0011.png

# ...

[TB] RESULT: DROWSY  (complexity proxy ready)

[TB] Simulation complete.
```

You can also call `scripts/run_tests.py` to run tests over a larger number of input images, with reporting on the success rate.

```sh
$ python scripts/run_tests.py

# ...

Test Summary:
Passed: 63 (63.00%)
Failed: 37
```
