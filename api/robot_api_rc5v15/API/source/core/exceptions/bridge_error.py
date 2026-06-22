from .base_api_error import ApiError


class BridgeError(ApiError):
    pass


class PipeError(BridgeError):
    pass


class PipeTimeoutError(BridgeError):
    pass


class PipeNameError(BridgeError):
    pass
