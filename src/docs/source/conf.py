import subprocess
import os

# Doxygen
subprocess.call("cd ../ && doxygen", shell=True)

# Configuration file for the Sphinx documentation builder.
#
# This file only contains a selection of the most common options. For a full
# list see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Path setup --------------------------------------------------------------

# If extensions (or modules to document with autodoc) are in another directory,
# add these directories to sys.path here. If the directory is relative to the
# documentation root, use os.path.abspath to make it absolute, like shown here.
#
# import os
# import sys
# sys.path.insert(0, os.path.abspath('.'))


# -- Project information -----------------------------------------------------

project = "Dephan ROS driver"
copyright = "2024, Anton Ledrov"
author = "Anton Ledrov"

# The full version, including alpha/beta/rc tags
release = "0.1.0"


# -- General configuration ---------------------------------------------------

# Add any Sphinx extension module names here, as strings. They can be
# extensions coming with Sphinx (named 'sphinx.ext.*') or your custom
# ones.
extensions = [
    "breathe",
    "sphinx_copybutton",
    "sphinx_tabs.tabs",
    "sphinx_rtd_theme",
    "sphinx_favicon",
]

# Add any paths that contain templates here, relative to this directory.
templates_path = ["_templates"]

# List of patterns, relative to source directory, that match files and
# directories to ignore when looking for source files.
# This pattern also affects html_static_path and html_extra_path.
exclude_patterns = []

# Breathe Configuration
breathe_projects = {
    "Dephan ROS driver": "../build/doxygen/xml",
}
breathe_default_project = "Dephan ROS driver"


# -- Options for HTML output -------------------------------------------------
html_context = {
    "current_version": os.environ["CURRENT_VERSION"],
    "versions": [
        ["docs", "https://dephan-llc.github.io/dephan-ros-driver/docs"],
        ["ROS:noetic", "https://dephan-llc.github.io/dephan-ros-driver/noetic"],
        ["ROS:iron", "https://dephan-llc.github.io/dephan-ros-driver/iron"],
    ],
}
# The theme to use for HTML and HTML Help pages.  See the documentation for
# a list of builtin themes.
#
html_theme = "sphinx_rtd_theme"

# Add any paths that contain custom static files (such as style sheets) here,
# relative to this directory. They are copied after the builtin static files,
# so a file named "default.css" will overwrite the builtin "default.css".
html_static_path = ["_static"]

# Tell sphinx what the primary language being documented is.
primary_domain = "cpp"

# Tell sphinx what the pygments highlight language should be.
highlight_language = "cpp"
