# python setup.py bdist_wheel
# sudo pip install ./dist/*.whl

from setuptools import setup, find_packages

with open("README.md", "r") as fh:
    README = fh.read()

requirements = [
    'opencv-contrib-python',
    'numpy'
]

setup_requirements = [
    # TODO: put setup requirements (distutils extensions, etc.) here
]

test_requirements = [
    'pytest'
]

setup(
    name="rp_server",
    version="1.0.0",
    
    description="CV Modules for Robot Semantic Mapping",
    long_description=README,
    
    url="https://github.com/hmz-15/Interactive-Scene-Reconstruction.git",
    
    packages=find_packages(
        exclude=["*.tests", "*.tests.*", "tests.*", "tests"]
    ),
    include_package_data=True,

    license="Apache-2.0",

    test_suite='tests',

    install_requires=requirements,
    
    tests_require=test_requirements,

    setup_requires=setup_requirements,

    classifiers=(
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.7",
        "License :: OSI Approved :: Apache-2.0 License",
        "Operating System :: OS Independent",
    ),
)