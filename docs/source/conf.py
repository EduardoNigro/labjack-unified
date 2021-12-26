# Configuration file for the Sphinx documentation builder.
#
# This file only contains a selection of the most common options. For a full
# list see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html
#
# Run this to build (from the package root directory):
# sphinx-build -b html docs/source docs/build/html

# -- Path setup --------------------------------------------------------------

# If extensions (or modules to document with autodoc) are in another directory,
# add these directories to sys.path here. If the directory is relative to the
# documentation root, use os.path.abspath to make it absolute, like shown here.
#
import os
import sys
sys.path.insert(0, os.path.abspath('..'))
sys.path.append(os.path.abspath('../..'))
import sphinx_rtd_theme


 # -- Project information -----------------------------------------------------

project = 'Labjack Unified'
copyright = '2021, Eduardo Nigro'
author = 'Eduardo Nigro'

# The full version, including alpha/beta/rc tags
release = '0.0.5'


# -- General configuration ---------------------------------------------------

extensions = [
    'sphinx.ext.intersphinx',
    'sphinx.ext.autodoc',
    'sphinx.ext.mathjax',
    'sphinx.ext.viewcode',    
    'sphinx_rtd_theme',
]
templates_path = ['_templates']
exclude_patterns = []
highlight_language='python3'
pygments_style = 'sphinx'

# -- Autodoc configuration ------------------------------------------------

autodoc_member_order = 'groupwise'

# -- Options for HTML output -------------------------------------------------

html_theme = 'sphinx_rtd_theme'
html_static_path = ['_static']
html_logo = '../images/logo.png'
html_favicon = '../images/favicon.png'
