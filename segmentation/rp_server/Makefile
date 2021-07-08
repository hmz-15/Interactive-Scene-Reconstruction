.PHONY: install clean dev dep

################################################################
# Environment Setup
################################################################

# install system dependencies
dep:
	pip install wheel


# install the rp_server Package
install:
	python setup.py bdist_wheel
	pip install ./dist/*.whl


# uninstall the rp_server Package
clean:
	pip uninstall rp_server
	rm -rf build *.egg-info *.bdist_wheel dist build


# install the rp_server Package (development mode)
dev:
	pip install -e .