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
        # Machine-generated constant tables (object-dictionary defs, register
        # maps, PDO wire layouts) — each constant gets its own stub page, which
        # is most of the ~11k generated pages and the reason the build OOMs /
        # runs for hours.  The constants stay in the headers; they are just
        # not rendered into the Sphinx API tree for now.
        EXCLUDE_PATTERNS       = */tests/* */test_* *_test.cpp *_test.h \
                                 */CiA*Defs.hpp */ETG*Defs.hpp */FSoEDefs.hpp \
                                 */Registers */Registers/* */*Registers.hpp \
                                 */drives/*/*PDO.hpp */sensors/*/*PDO.hpp \
                                 */drives/*/PdoSetup.hpp */drives/*Errors.hpp \
                                 */profiles/*/*Parameters.hpp \
                                 */ethercat/PDO.hpp
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

# -- Warning policy (CI builds with -W) ----------------------------------------

# Warnings that are pure generated-docs noise and cannot be fixed at the
# source (they live in exhale-generated api/ stubs, not in .md files):
# - duplicate_declaration: every nested class/struct is declared both on its
#   parent's page and on its own page — inherent to the unabridged API tree.
# - toc.not_included: unabridged-orphan stubs are intentionally not toctree'd.
# - docutils: unbalanced * / ` markup inside Doxygen comments leaks into the
#   generated .rst (hundreds of sites, cosmetic).
suppress_warnings = [
    'duplicate_declaration',
    'toc.not_included',
    'docutils',
    # fallback-parser notices on Doxygen-rendered signatures Sphinx's C++
    # grammar can't handle (digit separators, brace initializers, ...)
    'source_code_parser.cpp',
    # Pygments can't lex the box-drawing/arrow chars used in ASCII diagrams
    # inside doc comments and .md code blocks; it retries in relaxed mode
    # and renders them fine — the warning is noise.
    'misc.highlighting_failure',
    # untyped warnings tagged by the filter in setup() below
    'doxygen_artifact',
]

import logging
import re


def setup(app):
    """Tag untyped warnings emitted for Doxygen-rendering artifacts.

    Sphinx emits some warnings *without* a type/subtype, which makes them
    impossible to list in ``suppress_warnings``.  The generated API pages
    produce such warnings when Sphinx's C++ parser cannot parse a signature
    string that Doxygen rendered — e.g. doubled specifiers
    (``constexpr constexpr``), digit separators (``200 '000``), or brace
    initializers (``= {.x = 1}``) — or when Exhale generates two namespace
    pages whose labels differ only in case.  These are rendering artifacts,
    not doc bugs: the declaration still renders via the fallback path.

    This filter stamps a ``doxygen_artifact`` type on those records so the
    ``suppress_warnings`` entry above silences exactly them — everything
    else still trips ``-W``.
    """

    class TagDoxygenArtifacts(logging.Filter):
        _pattern = re.compile(
            r'Error when parsing function declaration'
            r'|Error in postfix expression'
            r'|Parsing of expression failed'
            r'|duplicate label'
        )

        def filter(self, record):
            if (record.levelno == logging.WARNING
                    and getattr(record, 'type', None) is None
                    and self._pattern.search(record.getMessage())):
                record.type = 'doxygen_artifact'
            return True

    # Must run *before* Sphinx's WarningSuppressor (which counts warnings
    # for -W), so insert at the front of the warning handler's filter chain.
    for handler in logging.getLogger('sphinx').handlers:
        handler.filters.insert(0, TagDoxygenArtifacts())

    # Furo renders the site navigation by calling the per-page ``toctree``
    # context callable with ``collapse=False, maxdepth=-1`` — i.e. the *full*
    # document tree — inside its own ``html-page-context`` handler (default
    # priority 500).  With ~10k generated api pages in the unabridged toctree
    # that is O(N) per page, O(N^2) overall — the write phase alone took
    # ~10 s/page (projected > 20 h).  Registering at a lower priority lets us
    # stub the callable for generated api pages before Furo sees it: the API
    # pages get an empty nav sidebar (a 10k-entry expanded tree is useless
    # there anyway) and hand-written pages keep the real tree.
    def _skip_global_nav_for_api_pages(app, pagename, templatename,
                                       context, doctree):
        if pagename.startswith('api/'):
            context['toctree'] = lambda **kwargs: ''

    app.connect('html-page-context', _skip_global_nav_for_api_pages,
                priority=400)

    # Breathe renders Doxygen's internal <ref> links as :ref: targets of the
    # form ``exhale_<kind>_<hash>``, but Exhale only emits labels for entities
    # that got their own page (documented members).  Links to undocumented
    # enum values / hidden members therefore dangle and emit ``ref.ref``
    # warnings on every generated file_* page.  Real ``:ref:`` targets in
    # hand-written docs never start with ``exhale_``, so resolving only those
    # to their display text (unlinked) keeps genuine broken references fatal
    # under -W.
    def _resolve_dangling_exhale_refs(app, env, node, contnode):
        if node.get('reftarget', '').startswith('exhale_'):
            return contnode
        return None

    app.connect('missing-reference', _resolve_dangling_exhale_refs)


# -- Extension configuration -------------------------------------------------
