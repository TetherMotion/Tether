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
    'exhale',
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

# Breathe configuration — the XML tree below is produced by Doxygen, which
# Exhale invokes automatically during the Sphinx build (exhaleExecutesDoxygen).
breathe_projects = {
    'Tether': str(Path(__file__).parent / '_doxygen' / 'xml')
}

breathe_default_project = 'Tether'

# Use Doxygen's XML output
breathe_domain_by_extension = {
    'h': 'cpp',
    'hpp': 'cpp',
    'c': 'c',
    'cpp': 'cpp',
}

# -- Options for Exhale extension --------------------------------------------

# Exhale runs Doxygen itself (exhaleExecutesDoxygen) and generates a .rst stub
# for every class/struct/enum/function under docs/api/, so no hand-maintained
# doxygenclass/doxygennamespace directives are needed. The generated tree is
# rooted at api/library_root.rst (referenced from index.rst).
exhale_args = {
    # These arguments are required
    'containmentFolder':    './api',
    'rootFileName':         'library_root.rst',
    'doxygenStripFromPath': '..',
    # Heavily encouraged optional arguments
    'rootFileTitle':        'Tether C++ API Reference',
    'createTreeView':       True,
    'fullToctreeMaxDepth':  1,
    # Toplevel page sections in the generated API tree
    'fullApiSubSectionTitle': 'API',
    # Exhale executes Doxygen during the Sphinx build — no separate step needed
    'exhaleExecutesDoxygen': True,
    'exhaleDoxygenStdin': '''
        INPUT                  = ../include/tether
        FILE_PATTERNS          = *.h *.hpp *.c *.cpp
        RECURSIVE              = YES
        EXCLUDE_PATTERNS       = */tests/* */test_* *_test.cpp *_test.h
        EXTRACT_ALL            = NO
        EXTRACT_PRIVATE        = NO
        EXTRACT_STATIC         = NO
        EXTRACT_LOCAL_CLASSES  = YES
        HIDE_UNDOC_MEMBERS     = YES
        HIDE_UNDOC_CLASSES     = YES
        INHERIT_DOCS           = YES
        GENERATE_HTML          = NO
        GENERATE_XML           = YES
        XML_OUTPUT             = xml
        XML_PROGRAMLISTING     = NO
        ENABLE_PREPROCESSING   = YES
        MACRO_EXPANSION        = YES
        EXPAND_ONLY_PREDEF     = YES
        SKIP_FUNCTION_MACROS   = YES
        SEARCH_INCLUDES        = YES
        INCLUDE_PATH           = ../include/tether
        HAVE_DOT               = NO
        NUM_PROC_THREADS       = 0
        WARNINGS               = YES
        WARN_IF_UNDOCUMENTED   = NO
        WARN_IF_DOC_ERROR      = YES
        WARN_NO_PARAMDOC       = NO
    ''',
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
