import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import detectors_ffi  # noqa: E402


@pytest.fixture(scope="session")
def lib(tmp_path_factory):
    """detectors.h compiled for the host, once per test session."""
    if detectors_ffi.find_compiler() is None:
        pytest.skip("no C++ compiler on PATH (set CXX)")
    path = detectors_ffi.build(str(tmp_path_factory.mktemp("detectors")))
    return detectors_ffi.load(path)


# Deterministic: same examples every run (repo policy), no example database
from hypothesis import settings  # noqa: E402

settings.register_profile("repo", derandomize=True, database=None)
settings.load_profile("repo")
