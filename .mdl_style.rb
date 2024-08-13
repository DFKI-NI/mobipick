# Style file for markdownlint

all

#exclude_rule 'fenced-code-language' # Fenced code blocks should have a language specified
exclude_rule 'first-line-h1' # First line in file should be a top level header
#exclude_rule 'first-header-h1'
exclude_rule 'ul-style'
exclude_rule 'no-multiple-blanks'
exclude_rule 'header-style'
#exclude_rule 'no-bare-urls'
#exclude_rule 'no-inline-html'

# Line length
rule 'line-length', :line_length => 120, :ignore_code_blocks => true, :tables => false

# Unordered list indentation
rule 'ul-indent', :indent => 2

# Ordered list item prefix
rule 'ol-prefix', :style => 'ordered'

# Multiple headers with the same content
rule 'no-duplicate-header', :allow_different_nesting => true
