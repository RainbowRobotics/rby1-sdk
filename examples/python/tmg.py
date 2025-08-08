import rby1_sdk as rby
import numpy as np
import matplotlib.pyplot as plt

tmg = rby.math.TrapezoidalMotionGenerator()

i = rby.math.TrapezoidalMotionGenerator.Input()
i.current_position = np.array([0.0, 0.0, 0.0])
i.current_velocity = np.array([0.0, 0.0, 0.0])
i.target_position = np.array([10.0, -5.5, -10.0])
i.velocity_limit = np.array([1.0, 5.0, 1.0])
i.acceleration_limit = np.array([0.5, 5.0, 0.1])
i.minimum_time = 5

tmg.update(i)

p = [tmg(t).position for t in np.arange(0, tmg.get_total_time(), 0.01)]

p = np.concatenate([p], axis=1)

plt.plot(np.arange(p.shape[0]) / 100., p)
plt.legend([f"Column {i}" for i in range(p.shape[1])])
plt.show()
