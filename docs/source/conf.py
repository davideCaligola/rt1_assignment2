# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

import os
import sys

import sphinx

print(sys.executable)

sys.path.insert(0, os.path.abspath("../.."))
sys.path.insert(1, os.path.abspath("../../rt1_ass2"))

project = 'rt2_ass1'
copyright = '2023, Davide'
author = 'Davide'
release = '0.0.1'

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = ["sphinx.ext.todo", "sphinx.ext.viewcode", "sphinx.ext.autodoc", "sphinx.ext.githubpages", "sphinx.ext.extlinks"]

extlinks = {"assignment2_2022":("https://github.com/davideCaligola/rt1_assignment2/tree/main/assignment_2_2022",
                                "assignment_2_2022")}

templates_path = ['_templates']
exclude_patterns = []

add_module_names = False

# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output


html_theme = 'sphinx_rtd_theme'
html_static_path = ['_static']

