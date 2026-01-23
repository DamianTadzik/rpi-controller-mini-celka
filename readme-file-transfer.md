# Transfer logs from Raspberry Pi to PC over SSH

## Requirements

- SSH access to RPi
- scp or rsync installed (default on Linux/macOS)

## Option 1 — scp (simplest)

**Copy entire logs/ directory to current folder on PC**
```bash
scp -r brzanpi@<RPI_IP>:/home/brzanpi/ws_minicelka/rpi-controller-mini-celka/logs .
```

**Copy to a specific directory on PC**
```bash
scp -r brzanpi@<RPI_IP>:/home/brzanpi/ws_minicelka/rpi-controller-mini-celka/logs ~/Downloads/
```

## Option 2 — rsync (recommended for large logs)

**One-time copy**
```bash
rsync -av --progress \
brzanpi@<RPI_IP>:/home/brzanpi/ws_minicelka/rpi-controller-mini-celka/logs/ \
./logs/
```

**Incremental copy (only new/changed files)**
```bash
rsync -av --progress \
brzanpi@<RPI_IP>:/home/brzanpi/ws_minicelka/rpi-controller-mini-celka/logs/ \
./logs/
```

## Notes

- Trailing `/` in `logs/` matters (copies contents, not directory wrapper)
- SSH uses port 22 by default (add `-p <port>` if different)
- Works while logger is running (Parquet files are append-safe)

## Example

```bash
scp -r \
brzanpi@192.168.137.134:/home/brzanpi/ws_minicelka/rpi-controller-mini-celka/logs \
/d/Dane/workspace/rpi-controller-mini-celka/
```

```bash
rsync -av --progress \
brzanpi@192.168.137.134:/home/brzanpi/ws_minicelka/rpi-controller-mini-celka/logs/ \
~/barka_logs/
```