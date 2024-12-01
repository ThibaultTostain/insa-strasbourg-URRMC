# -- Copy additional files (e.g., videos) to the build directory -------------
import os
import shutil

# Configuration file for the Sphinx documentation builder.

# -- Project information -----------------------------------------------------
project = 'Documentation URMC 2024'
copyright = '2024, INSA Promotion GE5 24-25'
author = 'INSA - GE5'

release = '1.0'
version = '1.0.0'

# -- General configuration ---------------------------------------------------
extensions = [
    'sphinx.ext.duration',
    'sphinx.ext.doctest',
    'sphinx.ext.autodoc',
    'sphinx.ext.autosummary',
    'sphinx.ext.intersphinx',
    'sphinx_copybutton',  # Pour ajouter un bouton "copier" aux blocs de code
]

# Mapping pour intersphinx
intersphinx_mapping = {
    'python': ('https://docs.python.org/3/', None),
    'sphinx': ('https://www.sphinx-doc.org/en/master/', None),
}
intersphinx_disabled_domains = ['std']

# Chemin vers les templates personnalisés
templates_path = ['_templates']

# -- Options for HTML output -------------------------------------------------
html_theme = 'sphinx_rtd_theme'

# Chemin vers les fichiers statiques (CSS, JS, images)
html_static_path = ['_static']

# Inclusion du fichier CSS personnalisé
html_css_files = [
    'custom.css',  # Nom du fichier CSS dans _static
]

# Options spécifiques pour le thème sphinx_rtd_theme
html_theme_options = {
    'collapse_navigation': False,  # Garde les sous-menus ouverts par défaut
    'navigation_depth': 4,         # Permet d'afficher 4 niveaux de navigation
}

# -- Options for EPUB output -------------------------------------------------
epub_show_urls = 'footnote'



def setup(app):
    app.connect("build-finished", copy_extra_files)

def copy_extra_files(app, exception):
    """Copies videos and other extra files to the output directory after building."""
    if exception is None:  # Execute only if the build was successful
        source_dir = os.path.abspath("simulation_epreuve_urmc/videos")
        dest_dir = os.path.join(app.outdir, "simulation_epreuve_urmc/videos")
        if os.path.exists(source_dir):
            shutil.copytree(source_dir, dest_dir, dirs_exist_ok=True)
