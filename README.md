# turtlebot_radio_emergency
Node to manage the emergency button


In turtlebot/turtlebot_bringup/param/mux.yaml must appear this in the subscribers:

  - name:        "Emergency button"
    topic:       "input/emergency_button"
    timeout:     0.2
    priority:    15
