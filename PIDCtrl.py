class PIDCtrl:
    def __init__(self,Kp,Ki,Kd,SetPoint):
        self.Kp = Kp # 比例系数
        self.Ki = Ki # 积分系数
        self.Kd = Kd # 微分系数
        self.SetPoint = SetPoint # 设置目标值
        self.Integral = 0 # 累计误差
        self.LastError = 0 # 上一次误差
        self.Output = 0 # 输出

    # 计算Output
    def PID_Compute(self,input):
        error = float(self.SetPoint) - float(input) # 误差
        self.Integral += error # 累计误差
        derivative = error - self.LastError # 积分误差
        self.Output = self.Kp * error + self.Ki * self.Integral + self.Kd * derivative # 计算输出
        self.LastError = error
        return self.Output

    # 设置目标值
    def PID_SetPoint(self,setpoint):
        self.SetPoint = float(setpoint)