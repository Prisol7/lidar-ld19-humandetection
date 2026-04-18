# Running `lidar-ld19` on a Raspberry Pi 5

This guide assumes:
- Raspberry Pi 5 running **Raspberry Pi OS 64-bit (Desktop)**.
- The image `lponce28/lidar-ld19:latest` has been pushed to Docker Hub **with an `arm64` build** (see the cross-build note at the bottom).
- An LD19 lidar plugged in over USB.

## 1. Install Docker (one-time)

```bash
sudo apt-get update
sudo apt-get install -y docker.io
sudo usermod -aG docker $USER
newgrp docker          # or log out and back in
docker ps              # should print an empty table, not a permission error
```

## 2. Plug in the lidar and find its device path

```bash
ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
```

Usually `/dev/ttyUSB0`. If nothing shows up, run `dmesg | tail` right after
plugging it in to see which device name the kernel assigned.

## 3. Pull the image

```bash
docker pull lponce28/lidar-ld19:latest
```

Verify it's actually `arm64`:

```bash
docker image inspect lponce28/lidar-ld19:latest | grep Architecture
```

You want `"Architecture": "arm64"`. If it says `amd64`, the image won't run on
the Pi — you'll get `exec format error`. See the cross-build note at the bottom.

## 4. Allow the container to draw on the Pi's desktop

Run this once per desktop session (it's reset on reboot):

```bash
xhost +local:docker
```

This tells the X server to trust local Docker clients. Without it, the window
can't open.

## 5. Run

```bash
docker run --rm -it \
  --device=/dev/ttyUSB0 \
  --group-add dialout \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v "$PWD/data":/data \
  -w /data \
  lponce28/lidar-ld19:latest
```

### What each flag does

| Flag | Purpose |
| --- | --- |
| `--rm` | Delete the container when it exits. |
| `-it` | Keep stdin open and attach a TTY so you can see logs and hit Ctrl-C. |
| `--device=/dev/ttyUSB0` | Expose the USB-serial lidar to the container. |
| `--group-add dialout` | Give the container process permission to open the serial device. |
| `-e DISPLAY=$DISPLAY` | Tell GUI apps inside which X display to use. |
| `-v /tmp/.X11-unix:/tmp/.X11-unix` | Share the X11 socket so the window can actually render. |
| `-v "$PWD/data":/data` | Persist `training_data.csv` (and, later, `human_rf.bin`) on the host. |
| `-w /data` | Run the binary with `/data` as its working directory so the CSV lands there. |

## 6. Calibrate, then use it

On startup the program spends **30 seconds** building a background map — keep
the scene still during that time. After calibration, motion points render in
red, clusters classified as human pick up a blue-to-red gradient and a smile,
and a `training_data.csv` file starts accumulating in `./data/` on the host.

Hit **ESC** or close the window to exit.

## Troubleshooting

**`exec format error` on `docker run`**
The image is for the wrong architecture. Rebuild with buildx (see below) or
build on the Pi itself.

**`cannot open display` / window never appears**
- Run `xhost +local:docker` (step 4).
- Confirm you're on Pi OS **Desktop**, not Lite. Headless Pi has no X server;
  the window will never open.
- Check `echo $DISPLAY` returns something like `:0`.

**`Failed to open /dev/ttyUSB0`**
- Run `ls /dev/ttyUSB*` — the path may differ (e.g. `/dev/ttyACM0`). Adjust
  the `--device=` flag accordingly.
- The binary has `/dev/ttyUSB0` hard-coded. If your device is different, either
  symlink it (`sudo ln -s /dev/ttyACM0 /dev/ttyUSB0`) or rebuild the image
  with `PORT` changed in `src/bin/main.rs`.

**Permission denied on the serial device**
You dropped `--group-add dialout`, or your Pi user isn't allowed to read
the device. Add it back; if it still fails, check `ls -l /dev/ttyUSB0` —
the group should be `dialout`.

**Container runs but no points appear**
- Verify the lidar LED is on and spinning.
- Check the logs for "Calibrating for 30s…". If that message never prints,
  the serial port isn't being read. Revisit the device-path and permission
  items above.

## Cross-build note (for whoever's publishing the image)

A Pi 5 needs an `arm64` image. From an `amd64` dev box, use buildx:

```bash
docker buildx create --name multi --use
docker buildx build \
  --platform linux/amd64,linux/arm64 \
  -t lponce28/lidar-ld19:latest \
  --push .
```

That publishes a multi-arch manifest, and `docker pull` on the Pi
automatically grabs the `arm64` variant.
