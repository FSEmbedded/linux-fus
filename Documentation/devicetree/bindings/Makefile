# SPDX-License-Identifier: GPL-2.0
DT_DOC_CHECKER ?= dt-doc-validate
DT_EXTRACT_EX ?= dt-extract-example
DT_MK_SCHEMA ?= dt-mk-schema

DT_SCHEMA_LINT = $(shell which yamllint)

DT_SCHEMA_MIN_VERSION = 2020.8.1

DT_TMP_SCHEMA := processed-schema.yaml

quiet_cmd_mk_schema = SCHEMA  $@
      cmd_mk_schema = f=$$(mktemp) ; \
                      $(if $(DT_MK_SCHEMA_FLAGS), \
                           echo $(real-prereqs), \
                           $(find_cmd)) > $$f ; \
                      $(DT_MK_SCHEMA) -j $(DT_MK_SCHEMA_FLAGS) @$$f > $@ ; \
		      rm -f $$f

define rule_chkdt
	$(if $(DT_SCHEMA_LINT),$(call cmd,yamllint),)
	$(call cmd,chk_bindings)
	$(call cmd,mk_schema)
endef

DT_DOCS = $(shell $(find_cmd) | sed -e 's|^$(srctree)/||')

override DTC_FLAGS := \
	-Wno-avoid_unnecessary_addr_size \
	-Wno-graph_child_address \
	-Wno-interrupt_provider

$(obj)/processed-schema-examples.json: $(DT_DOCS) $(src)/.yamllint check_dtschema_version FORCE
	$(call if_changed_rule,chkdt)

ifeq ($(CHECK_DTBS),)
extra-y += $(patsubst $(src)/%.yaml,%.example.dts, $(DT_SCHEMA_FILES))
extra-y += $(patsubst $(src)/%.yaml,%.example.dt.yaml, $(DT_SCHEMA_FILES))
endif

# Unless DT_SCHEMA_FILES is specified, use the full schema for dtbs_check too.
# Just copy processed-schema-examples.json

$(obj)/processed-schema.json: $(obj)/processed-schema-examples.json FORCE
	$(call if_changed,copy)

DT_SCHEMA_FILES = $(DT_DOCS)

else

# If DT_SCHEMA_FILES is specified, use it for processed-schema.json

$(obj)/processed-schema.json: DT_MK_SCHEMA_FLAGS := -u
$(obj)/processed-schema.json: $(DT_SCHEMA_FILES) check_dtschema_version FORCE
	$(call if_changed,mk_schema)

extra-y += $(DT_TMP_SCHEMA)
