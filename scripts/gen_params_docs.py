#!/usr/bin/env python3
"""Generate doc/parameters.md as markdown tables from the .yaml parameter schema.

In contrast to the stock `generate_parameter_library_markdown` CLI this renders one
table per parameter group and adds a `Consumed by` column.

The `Consumed by` cell is taken from the trailing `Consumed by:` marker in the
parameter's `description:` in the schema, so the schema stays the single source of
truth. Parameters without the marker get `none` in the column and a warning is
printed.

Usage:
  python3 scripts/gen_params_docs.py \
      --input_yaml_file config/teb_controller_parameters.yaml \
      --output_markdown_file doc/parameters.md
"""

import argparse
import sys

from generate_parameter_library_py.generate_markdown import (
    DefaultConfigMarkdown,
    ParameterValidationMarkdown,
)
from generate_parameter_library_py.parse_yaml import GenerateCode

CONSUMED_BY_MARKER = 'Consumed by:'


def escape(text):
    """Make text safe for a markdown table cell."""
    return text.replace('|', '\\|').replace('\n', '<br>')


def validation_text(param):
    parts = []
    for validation in param.parameter_validations:
        text = str(ParameterValidationMarkdown(validation)).strip().lstrip('- ').strip()
        parts.append(text)
    return '<br>'.join(parts)


def param_row(param):
    name = param.parameter_name
    description = param.parameter_description.strip()
    head, sep, consumed_by = description.rpartition(CONSUMED_BY_MARKER)
    if not sep:
        sys.stderr.write(
            '[WARN] %s: description lacks "%s" — defaulting to "none"\n'
            % (name, CONSUMED_BY_MARKER)
        )
        head, consumed_by = description, 'none'
    else:
        description, consumed_by = head.strip(), consumed_by.strip()

    constraints = validation_text(param)
    if param.parameter_read_only:
        constraints = ('(read-only)' + (('<br>' + constraints) if constraints else ''))

    return '| `{}` | `{}` | `{}` | {} | {} | {} |'.format(
        escape(name),
        param.code_gen_variable.defined_type,
        escape(param.code_gen_variable.lang_str_value),
        escape(constraints),
        escape(description),
        escape(consumed_by) or '—',
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input_yaml_file', required=True)
    parser.add_argument('--output_markdown_file', required=True)
    args = parser.parse_args()

    gen = GenerateCode('markdown')
    gen.parse(args.input_yaml_file, '')

    groups = {}
    for param in gen.declare_parameters:
        row = param_row(param)
        group = param.parameter_name.split('.')[1]
        groups.setdefault(group, []).append(row)

    lines = []
    lines.append('# Nav2 TEB Controller — Parameters')
    lines.append('')
    lines.append(
        '> **Auto-generated** from the parameter schema '
        '`config/teb_controller_parameters.yaml` (via `scripts/gen_params_docs.py`, '
        'invoked by `make docs`). Never edit this file by hand — change the schema '
        '`description:` fields instead.'
    )
    lines.append('')
    lines.append(
        'The same descriptions appear at runtime via `ros2 param describe`. The '
        '`Consumed by` column lists the code (edge / controller function) that reads '
        'each parameter; `—` marks parameters that are not read anywhere.'
    )
    lines.append('')
    lines.append('## Default Config')
    lines.append('')
    default_block = str(DefaultConfigMarkdown(gen))
    default_block = default_block.replace('Default Config\n', '', 1)
    lines.append(default_block)
    lines.append('')

    for group, group_rows in groups.items():
        lines.append('## `{}`'.format(group))
        lines.append('')
        lines.append('| Param | Type | Default | Constraints | Description | Consumed by |')
        lines.append('|---|---|---|---|---|---|')
        lines.extend(group_rows)
        lines.append('')

    with open(args.output_markdown_file, 'w') as f:
        f.write('\n'.join(lines))


if __name__ == '__main__':
    sys.exit(main())