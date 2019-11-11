class InvalidChainError(TypeError):
    def __init__(self, message="Invalid Kinematic chain, the Solver cannot load!"):
        self.message = message
