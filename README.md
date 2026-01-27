# QuadEncoder — micro:bit MakeCode Extension

A MakeCode extension for controlling the Yahboom 4‑channel quad encoder motor controller via I2C from a BBC micro:bit.

## Overview

This extension provides block-based and TypeScript APIs for:
- Controlling up to 4 motors with encoder feedback
- Reading encoder values (real-time and accumulated)
- Supporting 5 motor types: 520, 310, TT with encoder disk, TT DC geared, L-type 520
- Configuring motor parameters (dead zone, reduction ratio, wheel diameter)
- Speed control via encoder feedback or direct PWM

**Hardware**: Yahboom 4-port motor controller board  
**Communication**: I2C (address 0x26)  
**Reference**: https://www.yahboom.net/public/upload/upload-html/1740571339/Drive%20motor%20and%20read%20encoder-USART.html

## Project Links

- Repository: https://github.com/League-Microbit/quadencoder
- Docs site: https://league-microbit.github.io/quadencoder/

## Use as Extension

This repository can be added as an extension in MakeCode.

1. Open https://makecode.microbit.org/
2. Click New Project
3. Click Extensions under the gearwheel menu
4. Search for https://github.com/League-Microbit/quadencoder and import

## Edit this Project

To edit this repository in MakeCode:

1. Open https://makecode.microbit.org/
2. Click Import → Import URL
3. Paste https://github.com/League-Microbit/quadencoder and click Import

## Build & Deploy (local)

Using PXT directly:

```bash
pxt build
pxt deploy
pxt test
```

Or via the provided Makefile:

```bash
make build
make deploy
make test
```

---

## Legacy MakeCode instructions (kept for reference)

> Open this page at [https://ericbusboom.github.io/quadencoder/](https://ericbusboom.github.io/quadencoder/)

## Use as Extension

This repository can be added as an **extension** in MakeCode.

* open [https://makecode.microbit.org/](https://makecode.microbit.org/)
* click on **New Project**
* click on **Extensions** under the gearwheel menu
* search for **https://github.com/ericbusboom/quadencoder** and import

## Edit this project

To edit this repository in MakeCode.

* open [https://makecode.microbit.org/](https://makecode.microbit.org/)
* click on **Import** then click on **Import URL**
* paste **https://github.com/ericbusboom/quadencoder** and click import

#### Metadata (used for search, rendering)

* for PXT/microbit
<script src="https://makecode.com/gh-pages-embed.js"></script><script>makeCodeRender("{{ site.makecode.home_url }}", "{{ site.github.owner_name }}/{{ site.github.repository_name }}");</script>


> Open this page at [https://ericbusboom.github.io/quadencoder/](https://ericbusboom.github.io/quadencoder/)

## Use as Extension

This repository can be added as an **extension** in MakeCode.

* open [https://makecode.microbit.org/](https://makecode.microbit.org/)
* click on **New Project**
* click on **Extensions** under the gearwheel menu
* search for **https://github.com/ericbusboom/quadencoder** and import

## Edit this project

To edit this repository in MakeCode.

* open [https://makecode.microbit.org/](https://makecode.microbit.org/)
* click on **Import** then click on **Import URL**
* paste **https://github.com/ericbusboom/quadencoder** and click import

#### Metadata (used for search, rendering)

* for PXT/microbit
<script src="https://makecode.com/gh-pages-embed.js"></script><script>makeCodeRender("{{ site.makecode.home_url }}", "{{ site.github.owner_name }}/{{ site.github.repository_name }}");</script>
