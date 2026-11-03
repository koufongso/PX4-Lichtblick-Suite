// Flat config for ESLint 9. The template's .eslintrc.yaml no longer works, @lichtblick/eslint-plugin v2 only exports flat configs.
import lichtblick from "@lichtblick/eslint-plugin";
import globals from "globals";

export default [
  { ignores: ["dist", "node_modules", "scripts"] },
  ...lichtblick.configs.base,
  ...lichtblick.configs.react,
  ...lichtblick.configs.typescript.map((config) => ({ ...config, files: ["**/*.ts", "**/*.tsx"] })),
  {
    languageOptions: { globals: globals.browser },
    rules: {
      "react-hooks/exhaustive-deps": "error",
    },
  },
  {
    // node:test's describe/it return promises the test runner awaits itself
    files: ["test/**/*.ts"],
    rules: { "@typescript-eslint/no-floating-promises": "off" },
  },
];
