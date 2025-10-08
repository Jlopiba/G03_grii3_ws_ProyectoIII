from example_interfaces.srv import SetBool

# Dentro de la clase AutoTurtle
self.srv = self.create_service(SetBool, 'toggle_turtle', self.toggle_callback)

def toggle_callback(self, request, response):
    if request.data:
        self.running = True
        response.success = True
        response.message = 'Tortuga reanudada'
    else:
        self.running = False
        response.success = True
        response.message = 'Tortuga detenida'
    self.get_logger().info(response.message)
    return response

