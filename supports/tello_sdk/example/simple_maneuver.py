from supports.tello_sdk.Command import Command

tello_command = Command()

tello_command.command()
tello_command.takeoff()

tello_command.move('left', 50)
tello_command.move('cw', 360)
tello_command.move('forward', 50)

tello_command.land()






