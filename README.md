<h1 align="center">S.T.A.L.K.E.R. Artifact Detectors</h1>
<p align="center"><i>Principles of Computer Organisation & Embedded project</i></p>
<div align="center">
  <a href="">Daryna Berezniuk,</a>
<a href="https://github.com/Den-Kachanov">Denys Kachanov,</a>
  <a href="https://github.com/shyrokovapn-ai">Anastasiia Shyrokova</a>

<a href="https://github.com/DoktorTomato">Mentor: Ivan Shevchuk</a>

<a></a>
</div>  

---

This project is part of ***Principles of Computer Organisation*** and ***Embedded*** courses. It is a recreation of three artifact detectors from the game *S.T.A.L.K.E.R. 2: Heart of Chornobyl* using BLE and UWB technologies. Mentioned detectors are "**Echo**" detector (Відгук), "**Bear**" detector(Ведмідь) and "**Veles**" detector (Велес).

---

## Table of contents
- [Overview](#overview)
- [Features](#features)  _(in progress)_
- [Repository structure](#repository-structure) _(in progress)_
- [Bill of materials](#bill-of-materials) _(in progress)_
- [Firmware](#firmware) _(to be done)_
- [Usage](#usage) _(to be done)_
- [Documentation](#documentation) _(in progress)_
- [Contact & support](#contact--support)
- [Acknowledgments](#acknowledgments)

---

## Overview

### What is our project about?
The goal of this project is to create three functional “artifact” detectors based on the game *S.T.A.L.K.E.R. 2: Heart of Chornobyl*. The detectors work by providing a more precise location of an artifact as the user approaches it. Since the game’s world is fictional, in the context of our project, an artifact is BLE beacon/UWB board, a separate device that emits a signal that can be located (read more in [features](#features) section). Depending on the type of detector, its operating principle varies between raw BLE and UWB with PDoA technique. 

While this project is inspired by the game, our research will also be relevant in areas such as locating missing item that acts as a beacon.

### Detectors' properties
| Detector type  | Specification                                                                              |
|----------------|--------------------------------------------------------------------------------------------|
| Echo detector  | The closer we get to the “artifact,” the more frequently the indicator flashes and beeps.  |
| Bear detector  | Shows the direction to the nearest “artifact” and also indicates the approximate distance. |
| Veles detector | Clearly indicates the position of the “artifact” relative to the user on the screen.       |

### Project status
- **Current Version:** Proof of concept.
- **Development Status:** Artifact in development.
- **Last Updated:** 28.10.2026.

### Work distribution
- [Daryna Berezniuk](https://github.com/DarynaBerezniuk) &#8212; hardware & documenting;
- [Denys Kachanov](https://github.com/Den-Kachanov) &#8212; main software;
- [Anastasiia Shyrokova](https://github.com/shyrokovapn-ai) &#8212; design & software help.
---
## Features

This project involves creating three devices, each with its own unique features, that point to a specific beacon.

| Detector type | Technology used | Components needed |
|---------------|-----------------|-------------------|
|Echo detector| ||

### Hardware features
- To be done

### Software features
- To be done
### Design features
- To be done

---
## Repository structure
```
.
├── README.md
├── Results
│   └── media
├── Build
│   ├── BOM
│   ├── PCB
│   └── CAD
├── Firmware
└── Docs

```

### Directory Descriptions

- **Build/**: All files needed to physically construct the hardware:
    - **CAD/**: 3D models.
    - **PCB/**: Circuit board design files.
    - **BOM/**: Complete bill of materials.
- **Docs/**: All documents related to this project, such as reports.
- **Firmware/**: All code related to the project.
- **Results/**: Results of our work.
    - **media/**: pictures/videos of our results.

This section will be updated during active development.

## Bill of Materials (BOM)


### Component Tables

| Category        | Component        | Specification                   | Quantity | Estimated Cost | Notes                     |
|-----------------|------------------|---------------------------------|----------|----------------|---------------------------|
| **Electronics** |                  |                                 |          |                |                           |
|                 | Microcontroller  | ESP32 | 1        | -        |                           |
| **Other**       |                  |                                 |          |                |                           |
|                 | Power Supply     | -             | -      | -         |                           |

**Total Estimated Cost**: to be estimated.

---

## Firmware

To be done.

---

## Usage
To be done.

---

## Documentation
To be done.

---

## Contact & support

### Developers
- [Daryna Berezniuk](https://github.com/DarynaBerezniuk) &#8212; Robotics student &#8212; [berezniuk.pn@ucu.edu.ua](mailto:berezniuk.pn@ucu.edu.ua);
- [Denys Kachanov](https://github.com/Den-Kachanov) &#8212; Robotics student &#8212; [kachanov.pn@ucu.edu.ua](mailto:kachanov.pn@ucu.edu.ua);
- [Anastasiia Shyrokova](https://github.com/shyrokovapn-ai) &#8212; Computer Science student &#8212; [shyrokova.pn@ucu.edu.ua](mailto:shyrokova.pn@ucu.edu.ua).

---

## Acknowledgments

### Support

We thank our mentor for providing full support in this project:
- [Ivan Shevchuk](https://github.com/DoktorTomato) &#8212; Computer Science student &#8212; [shevchuk.pn@ucu.edu.ua](mailto:shevchuk.pn@ucu.edu.ua).


This project is supported by Applied Science Faculty of Ukrainian Catholic University as part of the ***Principles of Computer Organization*** and ***Embedded*** courses. 

### Inspiration & References
This project is inspired by the game *[S.T.A.L.K.E.R. 2: Heart of Chornobyl](https://en.wikipedia.org/wiki/S.T.A.L.K.E.R._2:_Heart_of_Chornobyl)*.