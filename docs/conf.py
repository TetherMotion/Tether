# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

import os
import sys
from pathlib import Path

# -- Path setup --------------------------------------------------------------

# If extensions (or modules to document with autodoc) are in another directory,
# add these directories to sys.path here.
project_root = Path(__file__).parent.parent.absolute()
sys.path.insert(0, str(project_root))

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

project = 'Tether'
copyright = '2026, Tether Contributors'
author = 'Tether Contributors'
release = '2.0.0'

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = [
    'breathe',
    'myst_parser',
]

templates_path = ['_templates']
exclude_patterns = ['Thumbs.db', '.DS_Store']

# The suffix(es) of source filenames.
source_suffix = {
    '.rst': 'restructuredtext',
    '.md': 'markdown',
}

# The master toctree document.
master_doc = 'index'

# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

html_theme = 'furo'
html_static_path = ['_static']

# Theme options
html_theme_options = {
    'source_repository': 'https://github.com/yourusername/ESP32EtherCAT',
    'source_branch': 'main',
    'source_directory': 'Tether/docs/',
    'sidebar_hide_name': False,
    'navigation_with_keys': True,
    'light_css_variables': {
        'font-stack': 'system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, "Helvetica Neue", Arial, sans-serif',
        'font-stack--monospace': 'Consolas, "Liberation Mono", Menlo, Courier, monospace',
    },
}

# Add any paths that contain custom static files (such as style sheets)
html_static_path = ['_static']

# -- Options for Breathe extension ------------------------------------------

# Breathe configuration
breathe_projects = {
    'Tether': str(project_root / 'build' / 'xml')
}

breathe_default_project = 'Tether'

# Use Doxygen's XML output
breathe_domain_by_extension = {
    'h': 'cpp',
    'hpp': 'cpp',
    'c': 'c',
    'cpp': 'cpp',
}

# -- Options for MyST-Parser -----------------------------------------------

# MyST configuration
myst_enable_extensions = [
    'colon_fence',
    'deflist',
    'fieldlist',
    'html_admonition',
    'html_image',
    'replacements',
    'smartquotes',
    'strikethrough',
    'substitution',
    'tasklist',
]

myst_enable_checkboxes = True
myst_heading_anchors = 3
myst_numfig = True

# -- Options for LaTeX output ------------------------------------------------

latex_elements = {
    'papersize': 'letterpaper',
    'pointsize': '10pt',
}

# Grouping the document tree into LaTeX files.
latex_documents = [
    (master_doc, 'Tether.tex', 'Tether Documentation',
     'Tether Contributors', 'manual'),
]

# -- Options for manual page output ----------------------------------------

man_pages = [
    (master_doc, 'tether', 'Tether Documentation',
     [author], 1)
]

# -- Options for Texinfo output ----------------------------------------------

texinfo_documents = [
    (master_doc, 'Tether', 'Tether Documentation',
     author, 'Tether', 'Modular C++ Library for EtherCAT Motion Control.',
     'Miscellaneous'),
]

# -- Extension configuration -------------------------------------------------
