from .base_api_error import ApiError


class ApiRuntimeError(ApiError):
    pass


class ApiValueError(ApiError):
    pass
