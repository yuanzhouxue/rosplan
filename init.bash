#!/usr/bin/env bash

echo "Checkout submodules..."
git submodule init
git submodule update
echo "Done."
echo "Updating rosplan dependencies, this may take a few minutes..."
cd src/ROSPlan
sed -i "s/github.com\/KCL-Planning/gitee.com\/yuanzhouxue/g" .gitmodules
sed -i "s/github.com\/gerardcanal\/PPDDL_determinization/gitee.com\/yuanzhouxue\/ppddl_parser/g" .gitmodules
sed -i "s/github.com\/gerardcanal/gitee.com\/yuanzhouxue/g" .gitmodules
sed -i "s/github.com\/ssanner/gitee.com\/yuanzhouxue/g" .gitmodules
sed -i "s/github.com\/oscar-lima/gitee.com\/yuanzhouxue/g" .gitmodules
sed -i "s/github.com\/sebastianstock/gitee.com\/yuanzhouxue/g" .gitmodules
git submodule sync
git submodule init
git submodule update
git commit -a "update submodule url"
echo "Done."
echo -e "\033[32m All things done, you can work further. \033[0m"