#!/usr/bin/env python3
"""
Тестовый файл для проверки связи между tests.py и roboticsForward.py
"""

import numpy as np
import matplotlib.pyplot as plt

# Импортируем функции из tests.py
from tests import RobotTrajectoryObj, q_of_t, qd_of_t, qdd_of_t, t_min, t_max

def test_trajectory_functions():
    """Тестируем функции траектории"""
    print("Тестирование функций траектории...")
    print(f"Временной диапазон: t_min = {t_min}, t_max = {t_max}")
    
    # Создаем объект траектории
    trajectory_obj = RobotTrajectoryObj()
    print(f"Объект траектории создан: t0 = {trajectory_obj.t0}, t_end = {trajectory_obj.t_end}")
    
    # Тестируем функции в нескольких точках времени
    test_times = np.linspace(t_min, t_max, 5)
    
    for t in test_times:
        q, qd, qdd = trajectory_obj.Evaluate(t)
        print(f"t = {t:.3f}: q = {q}, qd = {qd}, qdd = {qdd}")
    
    # Тестируем отдельные функции
    t_test = (t_min + t_max) / 2
    q_test = q_of_t(t_test)
    qd_test = qd_of_t(t_test)
    qdd_test = qdd_of_t(t_test)
    
    print(f"\nТест в средней точке t = {t_test:.3f}:")
    print(f"q_of_t: {q_test}")
    print(f"qd_of_t: {qd_test}")
    print(f"qdd_of_t: {qdd_test}")
    
    return True

def plot_trajectory():
    """Строим графики траектории"""
    print("\nСтроим графики траектории...")
    
    t_plot = np.linspace(t_min, t_max, 100)
    q_plot = np.array([q_of_t(t) for t in t_plot])
    qd_plot = np.array([qd_of_t(t) for t in t_plot])
    qdd_plot = np.array([qdd_of_t(t) for t in t_plot])
    
    fig, axes = plt.subplots(3, 1, figsize=(10, 8))
    
    # Позиции
    axes[0].plot(t_plot, q_plot[:, 0], label='qz')
    axes[0].plot(t_plot, q_plot[:, 1], label='q1')
    axes[0].plot(t_plot, q_plot[:, 2], label='q2')
    axes[0].plot(t_plot, q_plot[:, 3], label='q3')
    axes[0].set_ylabel('Position')
    axes[0].legend()
    axes[0].grid(True)
    
    # Скорости
    axes[1].plot(t_plot, qd_plot[:, 0], label='qzd')
    axes[1].plot(t_plot, qd_plot[:, 1], label='q1d')
    axes[1].plot(t_plot, qd_plot[:, 2], label='q2d')
    axes[1].plot(t_plot, qd_plot[:, 3], label='q3d')
    axes[1].set_ylabel('Velocity')
    axes[1].legend()
    axes[1].grid(True)
    
    # Ускорения
    axes[2].plot(t_plot, qdd_plot[:, 0], label='qzdd')
    axes[2].plot(t_plot, qdd_plot[:, 1], label='q1dd')
    axes[2].plot(t_plot, qdd_plot[:, 2], label='q2dd')
    axes[2].plot(t_plot, qdd_plot[:, 3], label='q3dd')
    axes[2].set_xlabel('Time [s]')
    axes[2].set_ylabel('Acceleration')
    axes[2].legend()
    axes[2].grid(True)
    
    plt.tight_layout()
    plt.savefig('trajectory_test.png', dpi=150, bbox_inches='tight')
    plt.show()
    
    print("Графики сохранены в trajectory_test.png")

if __name__ == "__main__":
    try:
        # Тестируем функции
        test_trajectory_functions()
        
        # Строим графики
        plot_trajectory()
        
        print("\n✅ Все тесты прошли успешно! Связь между файлами работает корректно.")
        
    except Exception as e:
        print(f"\n❌ Ошибка при тестировании: {e}")
        import traceback
        traceback.print_exc()
