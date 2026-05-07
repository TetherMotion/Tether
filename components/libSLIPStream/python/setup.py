"""
Setup configuration for slipstream Python package.
"""

from setuptools import setup, find_packages
from pathlib import Path

# Read the README file
readme_file = Path(__file__).parent / "README.md"
long_description = readme_file.read_text(encoding="utf-8") if readme_file.exists() else ""

setup(
    name="slipstream",
    version="1.0.0",
    description="Python library for SLIP frame monitoring and processing with CRC32 validation",
    long_description=long_description,
    long_description_content_type="text/markdown",
    author="Uli Köhler",
    author_email="github@techoverflow.net",
    url="https://github.com/ulikoehler/libSLIPStream",
    license="See LICENSE file",
    packages=find_packages(),
    scripts=["scripts/monitor_slip.py"],
    python_requires=">=3.6",
    install_requires=[
        # pyserial is optional but recommended for serial support
        # Include it as optional dependency in extras_require
    ],
    extras_require={
        "serial": ["pyserial>=3.5"],
        "dev": ["pytest>=6.0", "pytest-cov"],
    },
    classifiers=[
        "Development Status :: 5 - Production/Stable",
        "Intended Audience :: Developers",
        "Intended Audience :: System Administrators",
        "Topic :: Communications :: Serial",
        "Topic :: System :: Networking",
        "Topic :: Software Development :: Libraries :: Python Modules",
        "License :: OSI Approved :: MIT License",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.6",
        "Programming Language :: Python :: 3.7",
        "Programming Language :: Python :: 3.8",
        "Programming Language :: Python :: 3.9",
        "Programming Language :: Python :: 3.10",
        "Programming Language :: Python :: 3.11",
        "Operating System :: OS Independent",
    ],
    keywords="slip serial framing crc32 protocol monitoring networking",
)
