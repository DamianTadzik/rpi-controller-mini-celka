# Adding Services to systemd (Raspberry Pi)

1. Create a service file

Place it in /etc/systemd/system/:

sudo nano /etc/systemd/system/<name>.service


Minimal template:

[Unit]
Description=My Service
After=network.target

[Service]
Type=simple
User=pi
WorkingDirectory=/home/pi
ExecStart=/usr/bin/python3 /home/pi/script.py
Restart=on-failure

[Install]
WantedBy=multi-user.target

2. Enable service (start on boot)
sudo systemctl enable <name>.service

3. Control the service
sudo systemctl start <name>.service
sudo systemctl stop <name>.service
sudo systemctl restart <name>.service
sudo systemctl status <name>.service

4. View logs
journalctl -u <name>.service -f
journalctl -u my_can_interface.service -f
journalctl -u my_mini_celka_controller.service -f

5. Edit code without rebooting
Just modify your script; systemd will use the updated file next time you start the service.

6. Disable service (so it is no longer active)
sudo systemctl disable <name>.service
