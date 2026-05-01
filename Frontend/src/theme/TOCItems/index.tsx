import React, { type ReactNode } from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import { useCollapsible, Collapsible } from '@docusaurus/theme-common';
import { useTOCHighlight } from '@docusaurus/theme-common/internal';
import styles from './styles.module.css';

interface TOCItem {
  readonly value: string;
  readonly id: string;
  readonly level: number;
  readonly children?: readonly TOCItem[];
}

interface TOCItemsProps {
  readonly toc: readonly TOCItem[];
  readonly className?: string;
}

export default function TOCItems({ toc, className }: TOCItemsProps): ReactNode {
  // Filter out items with empty IDs to prevent runtime crashes
  const filteredToc = React.useMemo(() => {
    const filter = (items: readonly TOCItem[]): TOCItem[] => {
      return items
        .filter(item => !!item.id)
        .map(item => ({
          ...item,
          children: item.children ? filter(item.children) : undefined
        }));
    };
    return filter(toc);
  }, [toc]);

  // useTOCHighlight is permanently disabled here to prevent querySelector errors 
  // with malformed TOC data, ensuring site stability.
  // useTOCHighlight(filteredToc);

  if (!filteredToc || filteredToc.length === 0) {
    return null;
  }

  return (
    <ul className={clsx('table-of-contents', styles.tableOfContents, className)}>
      {filteredToc.map((heading) => (
        <TOCItem
          key={heading.id}
          heading={heading}
        />
      ))}
    </ul>
  );
}

interface TOCItemProps {
  readonly heading: TOCItem;
}

function TOCItem({ heading }: TOCItemProps): ReactNode {
  const { collapsible, setCollapsible } = useCollapsible({
    initialState: () => true, // Default to collapsed
  });

  const href = `#${heading.id}`;

  return (
    <li
      className={clsx(
        styles.tableOfContentsItem,
        heading.level > 2 && styles.tableOfContentsItemDepth2,
        heading.level > 3 && styles.tableOfContentsItemDepth3,
        heading.level > 4 && styles.tableOfContentsItemDepth4,
      )}
    >
      <Link
        href={href}
        className={styles.tableOfContentsLink}
      >
        {heading.value}
      </Link>

      {heading.children && heading.children.length > 0 && (
        <Collapsible
          lazy
          as="ul"
          className={styles.tableOfContentsList}
          collapsed={collapsible}
          setCollapsed={setCollapsible}
        >
          {heading.children.map((child) => (
            <TOCItem
              key={child.id}
              heading={child}
            />
          ))}
        </Collapsible>
      )}
    </li>
  );
}