import path from "node:path";
import fs from "node:fs";

const DIR_PATH = path.resolve();

const WHITE_LIST = [
  "index.md",
  ".vitepress",
  "node_modules",
  ".idea",
  "assets",
];

const isDirectory = (p) => fs.lstatSync(p).isDirectory();

// 辅助：过滤白名单和 _pic 文件夹
function shouldSkip(name, fullPath) {
  if (WHITE_LIST.includes(name)) return true;
  if (isDirectory(fullPath) && name.endsWith('_pic')) return true;
  return false;
}

function getList(params, dirPath, pathname) {
  const res = [];
  for (const file of params) {
    const fullPath = path.join(dirPath, file);
    if (shouldSkip(file, fullPath)) continue;

    if (isDirectory(fullPath)) {
      const subFiles = fs.readdirSync(fullPath);
      res.push({
        text: file,
        collapsible: true,
        items: getList(subFiles, fullPath, `${pathname}/${file}`),
      });
    } else {
      if (path.extname(file) !== '.md') continue;
      const baseName = path.basename(file, '.md');
      res.push({
        text: baseName,
        link: `${pathname}/${baseName}`,
      });
    }
  }
  return res;
}

/**
 * 生成以当前文件夹名为顶层标题的侧边栏结构
 * @param {string} pathname - 如 '/blogs/ROS'
 * @returns {Array} VitePress sidebar 格式
 */
export const set_sidebar = (pathname) => {
  // 标准化路径：去掉开头的 /
  const cleanPath = pathname.replace(/^\/+/, '');
  const absPath = path.join(DIR_PATH, cleanPath);

  // 获取当前文件夹名（如 ROS）
  const folderName = path.basename(cleanPath);

  // 读取该文件夹下的内容
  let files = [];
  try {
    files = fs.readdirSync(absPath);
  } catch (err) {
    console.warn(`[auto_sidebar] 路径不存在: ${absPath}`);
    return [];
  }

  // 过滤白名单和 _pic
  const filteredFiles = files.filter(file => {
    const fullPath = path.join(absPath, file);
    return !shouldSkip(file, fullPath);
  });

  // 获取内部 items
  const items = getList(filteredFiles, absPath, `/${cleanPath}`);

  // 返回包裹结构
  return [
    {
      text: folderName,
      collapsible: true,
      items: items,
    }
  ];
};