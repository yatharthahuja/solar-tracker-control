s=tf('s');
sys = 10/(0.004*s^2 + 0.2*s + 100)
pidTuner(sys)
