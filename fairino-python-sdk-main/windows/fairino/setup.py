# setup.py — unified build + install for fairino SDK
#
# Works with: Python 3.8–3.12
# Usage:
#   python setup.py build_ext --inplace      ← build Cython module
#   pip install .                            ← install fairino as a package

from setuptools import setup, Extension, find_packages
from Cython.Build import cythonize
import sys

# Define your Cython extension
ext_modules = cythonize(
    [
        Extension(
            name="fairino.Robot",           # module path
            sources=["Robot.py"],           # the source file
            language="c",                   # target C code
        )
    ],
    compiler_directives={"language_level": "3"},
)

# Package setup
setup(
    name="fairino",
    version="1.0.0",
    author="Fairino Robotics",
    description="Python SDK for Fairino robot control",
    packages=find_packages(),   # auto-detect __init__.py packages
    ext_modules=ext_modules,    # include the compiled Robot module
    python_requires=">=3.8",
)
