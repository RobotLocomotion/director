import os
import coverage
cov = coverage.coverage(config_file=os.environ['COVERAGE_PROCESS_START'])
cov.combine()
cov.html_report()
