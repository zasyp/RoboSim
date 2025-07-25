import numpy as np

class GreyPIDController:
    def __init__(self, Kp0, Ki0, Kd0, window=5):
        self.Kp, self.Ki, self.Kd = Kp0, Ki0, Kd0
        self.errors = []
        self.window = window
        self.integral = 0.0
        self.last_error = 0.0
        self.last_time = None

    def update(self, error, d_error, t):
        # Буфер последних ошибок для GM(1,1)
        self.errors.append(error)
        if len(self.errors) > self.window:
            self.errors.pop(0)

        # Автоматическая GM(1,1)-подстройка Kp, Ki, Kd
        if len(self.errors) >= 4:
            import numpy as _np
            # 1) cumsum
            x1 = _np.cumsum(_np.array(self.errors))
            # 2) средний ряд
            z = 0.5*(x1[1:] + x1[:-1])
            # 3) собираем СЛАУ B * [a,b]^T = Y
            B = _np.column_stack([-z, _np.ones_like(z)])
            Y = _np.array(self.errors[1:])
            # 4) наименьшие квадраты
            [[a], [b]] = _np.linalg.lstsq(B, Y, rcond=None)[0].reshape(2,1)
            # 5) можно скорректировать Kd, Kp, Ki на основе a, b
            #    например: Kd = baseKd * (1 + alpha * a)
            alpha = 0.5
            self.Kd *= (1 + alpha * abs(a))
            #    аналогично Kp, Ki, если нужно

        # Собираем PID-выход
        if self.last_time is None:
            dt = 0.0
        else:
            dt = t - self.last_time

        self.last_time = t
        self.integral += error * dt

        u = self.Kp * error + self.Ki * self.integral + self.Kd * d_error
        self.last_error = error
        return u