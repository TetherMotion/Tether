"""
Tether Python Package Setup

To install in development mode:
    pip install -e .

To build wheels:
    pip wheel . --no-deps
"""

from setuptools import setup, find_packages
from pathlib import Path

# Read long description from README
this_directory = Path(__file__).parent
long_description = ""
readme_path = this_directory / "README.md"
if readme_path.exists():
    long_description = readme_path.read_text()

setup(
    name="tether",
    version="2.0.0",
    author="Tether Development Team",
    description="EtherCAT and G-code library with Python bindings",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/your-org/tether",
    packages=find_packages(where="src"),
    package_dir={"": "src"},
    classifiers=[
        "Development Status :: 4 - Beta",
        "Intended Audience :: Developers",
        "Intended Audience :: Manufacturing",
        "License :: OSI Approved :: MIT License",
        "Operating System :: POSIX :: Linux",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.8",
        "Programming Language :: Python :: 3.9",
        "Programming Language :: Python :: 3.10",
        "Programming Language :: Python :: 3.11",
        "Programming Language :: Python :: 3.12",
        "Programming Language :: C++",
        "Topic :: Scientific/Engineering",
        "Topic :: System :: Hardware",
    ],
    python_requires=">=3.8",
    install_requires=[
        "numpy>=1.20",
    ],
    extras_require={
        "dev": [
            "pytest>=7.0",
            "pytest-cov",
            "black",
            "isort",
            "mypy",
        ],
        "docs": [
            "sphinx",
            "sphinx-rtd-theme",
        ],
    },
    # Native modules are built separately via CMake and placed in package
    include_package_data=True,
    package_data={
        "tether": ["*.so", "*.pyd", "*.dll"],
    },
)
