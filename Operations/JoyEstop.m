[axes, buttons, povs] = read(joy);
if buttons(4) == 1
    error("EStop Detected, Exiting program")
end