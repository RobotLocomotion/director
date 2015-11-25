Generating a module graph
=========================

You can use the tool `snakefood <http://furius.ca/snakefood/>`_ to inspect
python files and generate a module dependency graph.  The snakefood documentation
has a good getting started guide.  You can install snakefood with pip::

    pip install snakefood


Or if you prefer apt-get on Ubuntu::

    sudo apt-get install snakefoot

Now you should have the ``sfood`` program available at the command line.
Here is an example command line to generate a recursive dependency graph,
ignoring external modules (only graphs director modules)::

    sfood -i --recursive src/python/director/visualization.py | sfood-graph | dot -Tpdf -o deps.pdf

This produces the output file deps.pdf.  It requires that ``dot`` is also installed.
