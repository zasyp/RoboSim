#!/usr/bin/env python3
"""
Script to check if all required dependencies are installed.
"""

import importlib
import sys

def check_dependency(module_name, package_name=None):
    """Check if a module can be imported."""
    if package_name is None:
        package_name = module_name
    
    try:
        importlib.import_module(module_name)
        print(f"✓ {package_name}")
        return True
    except ImportError:
        print(f"✗ {package_name} - NOT FOUND")
        return False

def main():
    print("=== Checking Dependencies ===")
    print()
    
    dependencies = [
        ("numpy", "NumPy"),
        ("matplotlib", "Matplotlib"),
        ("scipy", "SciPy"),
        ("exudyn", "Exudyn"),
        ("nurbspy", "nurbspy"),
        ("sympy", "SymPy"),
        ("autograd", "Autograd"),
    ]
    
    all_installed = True
    for module_name, package_name in dependencies:
        if not check_dependency(module_name, package_name):
            all_installed = False
    
    print()
    if all_installed:
        print("✓ All dependencies are installed!")
        print("You can run the simulation.")
    else:
        print("✗ Some dependencies are missing.")
        print("Please install missing packages using:")
        print("pip install -r ../requirements.txt")
    
    return all_installed

if __name__ == "__main__":
    main() 