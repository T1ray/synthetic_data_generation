from pathlib import Path

import pytest

from synthetic_data_generation.bag_roundtrip import process_message
from synthetic_data_generation.bag_roundtrip import prepare_output_path
from synthetic_data_generation.smoke_test import run_smoke_test


def test_process_message_is_identity():
    message = object()
    assert process_message("/topic", message, 123) is message


def test_roundtrip_smoke():
    run_smoke_test()


def test_existing_output_is_never_deleted(tmp_path: Path):
    output = tmp_path / "existing_output"
    output.mkdir()
    important = output / "important.txt"
    important.write_text("must survive", encoding="utf-8")

    with pytest.raises(FileExistsError, match="already exists"):
        prepare_output_path(output)

    assert output.is_dir()
    assert important.read_text(encoding="utf-8") == "must survive"

