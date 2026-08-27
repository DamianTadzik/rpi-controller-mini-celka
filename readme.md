# Runtime controller, observer loop, CAN interface and logger

## USEFUL COMMANDS
#### Temporarly
sudo systemctl start minicelka_runtime.service
sudo systemctl stop minicelka_runtime.service
sudo systemctl restart minicelka_runtime.service
#### Autoboot
sudo systemctl enable minicelka_runtime.service
sudo systemctl disable minicelka_runtime.service
#### Status
journalctl -u minicelka_runtime.service -f
systemctl status minicelka_runtime.service

#### python
source /home/brzanpi/ws_minicelka/venv/bin/activate
python /home/brzanpi/ws_minicelka/rpi-controller-mini-celka/src/log_rotate.py
