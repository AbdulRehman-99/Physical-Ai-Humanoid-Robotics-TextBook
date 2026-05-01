import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import styles from './styles.module.css';

interface DocSidebarItem {
  type: string;
  label: string;
  href?: string;
  to?: string;
  children?: DocSidebarItem[];
  collapsed?: boolean;
  collapsible?: boolean;
  active?: boolean;
}

interface DocSidebarProps {
  sidebar: any[];
  path: string;
  onCollapse: () => void;
  isHidden: boolean;
  className?: string;
}

const DocSidebar: React.FC<DocSidebarProps> = ({ sidebar = [], className }) => {
  const items = sidebar;
  const [expandedCategories, setExpandedCategories] = React.useState<Record<string, boolean>>({});

  const toggleCategory = (label: string) => {
    setExpandedCategories(prev => ({
      ...prev,
      [label]: !prev[label]
    }));
  };

  const renderNavItem = (item: any, depth = 0) => {
    const isLink = item.type === 'link';
    const isCategory = item.type === 'category';
    const hasChildren = item.items && item.items.length > 0;
    const isExpanded = expandedCategories[item.label] || false;
    const href = item.href;
    const isActive = item.active;

    return (
      <li
        key={`${item.label}-${depth}-${Math.random()}`}
        className={clsx(
          styles.sidebarItem,
          isLink && styles.sidebarItemLink,
          isCategory && styles.sidebarItemCategory,
          depth > 0 && styles.sidebarItemNested,
          isActive && styles.sidebarItemActive,
          isExpanded && styles.sidebarItemExpanded
        )}
      >
        {isLink ? (
          <Link
            to={href || '#'}
            className={clsx(
              styles.sidebarLink,
              isActive && styles.sidebarLinkActive
            )}
          >
            <span className={styles.sidebarLinkText}>{item.label}</span>
          </Link>
        ) : (
          <div 
            className={styles.sidebarCategory}
            onClick={() => toggleCategory(item.label)}
            style={{ cursor: 'pointer' }}
          >
            <span className={styles.sidebarCategoryText}>{item.label}</span>
            {hasChildren && (
              <span className={clsx(styles.categoryArrow, isExpanded && styles.categoryArrowExpanded)}>
                ▶
              </span>
            )}
          </div>
        )}

        {hasChildren && isExpanded && (
          <ul className={styles.sidebarSublist}>
            {item.items.map((child: any) => renderNavItem(child, depth + 1))}
          </ul>
        )}
      </li>
    );
  };

  return (
    <aside className={clsx('theme-doc-sidebar', 'sidebar', styles.sidebar, className)} role="complementary">
      <nav className={styles.sidebarNav} aria-label="Document navigation">
        <ul className={styles.sidebarList}>
          {items && items.length > 0 ? (
            items.map((item) => renderNavItem(item))
          ) : (
            <li className={styles.sidebarItem}>
              <span className={styles.sidebarPlaceholder}>No sidebar items available</span>
            </li>
          )}
        </ul>
      </nav>
    </aside>
  );
};

export default DocSidebar;