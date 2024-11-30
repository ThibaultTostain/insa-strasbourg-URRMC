# Configuration file for the Sphinx documentation builder.

# -- Project information -----------------------------------------------------

project = 'Lumache'  # Nom du projet
copyright = '2021, Graziella'  # Informations de copyright
author = 'Graziella'  # Auteur de la documentation

# Version et release du projet
release = '0.1'
version = '0.1.0'

# -- General configuration ---------------------------------------------------

# Extensions utilisées par Sphinx
extensions = [
    'sphinx.ext.duration',  # Mesure le temps de construction
    'sphinx.ext.doctest',  # Test des blocs de code dans la documentation
    'sphinx.ext.autodoc',  # Génère automatiquement la documentation des modules
    'sphinx.ext.autosummary',  # Génère automatiquement des résumés
    'sphinx.ext.intersphinx',  # Lien vers d'autres documentations
    'sphinx_copybutton',  # Ajoute un bouton "copier" sur les blocs de code
]

# Configuration de intersphinx pour le lien vers d'autres documentations
intersphinx_mapping = {
    'python': ('https://docs.python.org/3/', None),  # Documentation Python
    'sphinx': ('https://www.sphinx-doc.org/en/master/', None),  # Documentation Sphinx
}
intersphinx_disabled_domains = ['std']  # Désactiver certains domaines par défaut

# Chemin vers les templates personnalisés
templates_path = ['_templates']

# -- Options for HTML output -------------------------------------------------

# Thème pour la documentation HTML
html_theme = 'sphinx_rtd_theme'

# Options spécifiques pour le thème sphinx_rtd_theme
html_theme_options = {
    'collapse_navigation': False,  # Garder les sous-menus ouverts
    'navigation_depth': 4,         # Profondeur de la navigation dans la barre latérale
    'style_external_links': True,  # Ajouter une icône aux liens externes
}

# -- Options for EPUB output -------------------------------------------------

# Format d'affichage des URLs dans les fichiers EPUB
epub_show_urls = 'footnote'

# -- Options supplémentaires (facultatif) ------------------------------------

# Configuration des chemins (par exemple pour ajouter des dossiers de modules)
# import os
# import sys
# sys.path.insert(0, os.path.abspath('../src'))
