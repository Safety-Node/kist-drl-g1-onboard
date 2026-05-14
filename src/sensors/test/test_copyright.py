# Copyright 2024 KIST DRL.
# Licensed under the Apache License, Version 2.0.
from ament_copyright.main import main
import pytest


# Skip copyright check until proper notices are in place.
@pytest.mark.skip(reason='TODO(infra): enable once copyright notices are settled.')
@pytest.mark.copyright
@pytest.mark.linter
def test_copyright():
    rc = main(argv=['.', 'test'])
    assert rc == 0, 'Found errors'
