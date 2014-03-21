Instructions for using python coverage


To enable python coverage:

1) prepend this directory to your PYTHONPATH environment variable

    This makes python import usercustomize.py whenever python starts up.
    The usercustomize.py script will start coverage if the environment
    variable COVERAGE_PROCESS_START is defined.

2) Set the environment variable COVERAGE_PROCESS_START to the path of coverage.conf

    coverage.conf is a configured file located in the project's build directory.


To save the html coverage report, run the following command:

   python report_html.py
